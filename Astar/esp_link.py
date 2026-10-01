"""Drive the esp32_4axis controller along a path planned by astar_core.

The firmware speaks one line-based protocol, over USB serial and over the WiFi
WebSocket on port 81 (this module uses the WebSocket, like stepper_gui.html; see
esp32_4axis/README.md): `M <axis> <steps>` moves an axis to an absolute step
position, and a JSON status line arrives every 250 ms. Each axis runs on its
own, so plain `M` commands to a far waypoint would NOT travel the straight line
A* planned - whichever axis is quickest would arrive first and the tool would
cut across a box. Instead the host plans the tool's speed along the whole path
and streams it, every 50 ms, with the firmware's `MS <axis> <pos> <hz> <acc>`
command: each axis is told its share of the tool speed right now, aimed at the
end of its current one-direction stretch. FastAccelStepper takes a new MS
mid-move without stopping, so the tool runs through the corners of the path
continuously (see follow()): it slows down for each corner just enough that no
axis has to change speed by more than it can within CORNER_TOL_CM of the
line, and only comes to rest at the end and at the vertices it is told to
stop at (a pick or drop sequence, say). Speeds are open loop between status
lines; each 250 ms status line then nudges every axis back toward where the
plan has it (TRACK_GAIN), since the axes lag their speed changes by different
amounts and that difference is what would take the tool off the line.

Tool point (x, y, z) cm  ->  firmware axis, via AXIS_MAP below.

    ASSUMPTIONS TO CHECK BEFORE FIRST USE (they are guesses from the travel
    lengths in the esp32_4axis README, not measured):
      * x = axis 3 (148.5 cm), y = axis 1 (gantry, 1+2 move together, 100.8 cm),
        z = axis 4 (28 cm)
      * step 0 of every axis (after H / Z) is tool coordinate 0, and a positive
        step is a positive tool direction. Flip `sign` for an axis that runs
        the wrong way; add `offset_cm` if the homed zero is not the arena
        origin.

Needs websocket-client (pip install websocket-client).
"""

import json
import math
import threading
import time
from dataclasses import dataclass

import websocket  # pip install websocket-client

WS_PORT = 81
DEFAULT_SPEED_CM_S = 50.0  # tool speed along the path
SPEED_FRACTION = 0.7      # of each axis's configured `run` speed
ACCEL_FRACTION = 0.5      # of each axis's configured `acc` the plan uses; the
                          # rest is kept spare to absorb corners and lag
SETTLE_TOL_CM = 0.3       # how close counts as "arrived"
CORNER_TOL_CM = 0.1       # most the tool may stray off the path rounding a corner
MIN_AXIS_SPEED_CM_S = 0.05  # floor on a commanded axis speed (the feedback keeps it from crawling)
TICK_S = 0.05             # streaming period
TRACK_GAIN = 2.0          # 1/s: speed added per cm an axis is off the plan (status feedback)
SETTLE_TIMEOUT_S = 15.0
START_TOL_CM = 0.5        # the machine must already be this close to the path start


@dataclass(frozen=True)
class AxisMap:
    axis: int             # firmware axis, 1-4 (axis 1 also drives its gantry partner 2)
    sign: int = 1         # +1 or -1: which way positive tool motion turns the axis
    offset_cm: float = 0.0  # tool coordinate at step 0


AXIS_MAP = {
    "x": AxisMap(3),
    "y": AxisMap(1),
    "z": AxisMap(4),
}
ORDER = ("x", "y", "z")


class LinkError(Exception):
    pass


class EspLink:
    def __init__(self, axis_map=None, log=print):
        self.map = axis_map or AXIS_MAP
        self.log = log
        self.ws = None
        self.status = None
        self._lock = threading.Lock()
        self._reader = None
        self._status_t = 0.0
        self._last = {}
        self._stop = threading.Event()   # set by stop(): abort a follow in progress

    # ---- connection -----------------------------------------------------

    def connect(self, host):
        """Open ws://<host>:81, the same socket stepper_gui.html uses. `host` is
        the board's hostname (stepper.local) or IP address."""
        host = host.strip() or "stepper.local"
        try:
            ws = websocket.create_connection(f"ws://{host}:{WS_PORT}", timeout=5)
        except (OSError, websocket.WebSocketException) as e:
            raise LinkError(f"cannot reach ws://{host}:{WS_PORT} - {e}")
        ws.settimeout(0.5)
        self.ws = ws
        self.status = None
        self._reader = threading.Thread(target=self._read_loop, args=(ws,), daemon=True)
        self._reader.start()
        self.send("V 1")
        self.send("?")
        t = time.time()
        while self.status is None:
            if time.time() - t > 3:
                self.close()
                raise LinkError(f"connected to {host} but no status came back")
            time.sleep(0.05)

    def close(self):
        ws, self.ws = self.ws, None
        if ws:
            try:
                ws.close()
            except (OSError, websocket.WebSocketException):
                pass

    @property
    def connected(self):
        return self.ws is not None

    def send(self, line):
        if not self.connected:
            raise LinkError("not connected")
        try:
            with self._lock:
                self.ws.send(line)
        except (OSError, websocket.WebSocketException, AttributeError) as e:
            self.close()
            raise LinkError(f"link lost: {e}")

    def _read_loop(self, ws):
        # recv() also answers the board's 2 s heartbeat pings, which it needs
        # to see or it drops the client.
        while self.ws is ws:
            try:
                line = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            except (OSError, websocket.WebSocketException):
                break
            if isinstance(line, bytes):
                line = line.decode(errors="replace")
            line = line.strip()
            if line.startswith("{"):
                try:
                    self.status = json.loads(line)
                    self._status_t = time.time()
                except ValueError:
                    pass
            elif line:
                self.log(line)
        if self.ws is ws:
            self.ws = None
            self.log("# link lost")

    def status_age(self):
        return time.time() - self._status_t

    # ---- unit conversion --------------------------------------------------

    def _axis(self, name):
        if not self.status:
            raise LinkError("no status yet")
        return self.status["axis"][self.map[name].axis - 1]

    def _spm(self, name):
        return self._axis(name)["c"]["spm"]

    def steps_to_cm(self, name, steps):
        m = self.map[name]
        return m.sign * steps / self._spm(name) * 100.0 + m.offset_cm

    def cm_to_steps(self, name, cm):
        m = self.map[name]
        return round(m.sign * (cm - m.offset_cm) / 100.0 * self._spm(name))

    def position_cm(self):
        """The tool point (x, y, z) in cm, from the firmware's step counts."""
        return tuple(self.steps_to_cm(n, self._axis(n)["pos"]) for n in ORDER)

    def max_speed_cm_s(self, name):
        run = self._axis(name)["c"]["run"]
        return SPEED_FRACTION * run / self._spm(name) * 100.0

    def jog(self, name, delta_cm):
        """Move axis `name` (x/y/z) by delta_cm now, at its configured jog
        speed - a plain relative J, not part of any route. The firmware fans
        it out to a gantry partner itself and clamps it to the soft limits."""
        m = self.map[name]
        steps = round(m.sign * delta_cm / 100.0 * self._spm(name))
        if steps:
            self.send(f"J {m.axis} {steps}")

    # ---- checks -----------------------------------------------------------

    def ready_problems(self):
        """Reasons the machine cannot follow a path right now (empty = ready)."""
        s = self.status
        if not s:
            return ["no status from the controller"]
        out = []
        if s["estop"]:
            out.append("e-stop is engaged")
        if not s["drives"]:
            out.append("drives are off (EN 1)")
        for n in ORDER:
            a = self._axis(n)
            # "idle" is legitimate: axis 4 has no home switch and is zeroed with
            # `Z 4`, and a zeroed axis reports idle, not homed. The status line
            # cannot tell that from a never-referenced axis, so referencing
            # every axis first is on the operator. Only refuse mid-homing/fault.
            if a["state"] not in ("homed", "idle"):
                out.append(f"{n} (axis {self.map[n].axis}) is '{a['state']}'")
        return out

    def limit_problems(self, points):
        """Reasons any point in `points` falls outside a firmware soft limit."""
        out = []
        for n in ORDER:
            c = self._axis(n)["c"]
            if c["lmax"] <= c["lmin"]:
                continue
            steps = [self.cm_to_steps(n, p[ORDER.index(n)]) for p in points]
            lo, hi = c["lmin"] * c["spm"] // 1000, c["lmax"] * c["spm"] // 1000
            if min(steps) < lo or max(steps) > hi:
                out.append(f"{n} path leaves axis {self.map[n].axis}'s soft limits "
                           f"({c['lmin'] / 10:g} to {c['lmax'] / 10:g} cm)")
        return out

    # ---- motion -----------------------------------------------------------

    def stop(self):
        self._stop.set()
        if self.connected:
            self.send("S")

    def _command(self, point):
        # only send an axis whose whole-step target actually changed, so a
        # resting axis is not re-commanded every tick
        for n in ORDER:
            steps = self.cm_to_steps(n, point[ORDER.index(n)])
            if self._last.get(n) != steps:
                self._last[n] = steps
                self.send(f"M {self.map[n].axis} {steps}")

    def follow(self, path, speed_cm_s=DEFAULT_SPEED_CM_S, progress=None, stops=()):
        """Run the tool along `path` (list of (x, y, z) cm) in one continuous
        motion, rounding each corner (within CORNER_TOL_CM) instead of
        stopping at it. `stops`: indices of path vertices to come to rest
        at exactly - each stretch between them is a continuous run of its
        own, and the machine is checked to have arrived before the next.
        Blocks until it arrives, is stopped, or fails. `progress(point)` gets
        the tool position, dead-reckoned from the planned motion, about 20
        times a second. Raises LinkError if the machine is not ready or the
        path is unusable."""
        if len(path) < 2:
            raise LinkError("path is empty")
        problems = self.ready_problems() + self.limit_problems(path)
        if problems:
            raise LinkError("; ".join(problems))
        here = self.position_cm()
        if math.dist(here, path[0]) > START_TOL_CM:
            raise LinkError("the machine is at (%.1f, %.1f, %.1f), not the path "
                            "start (%.1f, %.1f, %.1f). Move it there first - a "
                            "straight run to the start is not collision-checked."
                            % (*here, *path[0]))
        self._stop.clear()
        cuts = sorted({i for i in stops if 0 < i < len(path) - 1})
        bounds = [0] + cuts + [len(path) - 1]
        self.log(f"# following {len(path)} points continuously, resting at "
                 f"{len(cuts)} vertex(es) on the way")
        for i, j in zip(bounds, bounds[1:]):
            self._run(path[i:j + 1], speed_cm_s, progress)

    def _axis_limits(self, n):
        """(steps per cm, max path speed cm/s, planning accel cm/s^2, full
        firmware accel cm/s^2) for axis `n`."""
        c = self._axis(n)["c"]
        spc = c["spm"] / 100.0
        return (spc, SPEED_FRACTION * c["run"] / spc, ACCEL_FRACTION * c["acc"] / spc,
                c["acc"] / spc)

    def _run(self, pts, speed_cm_s, progress):
        """One continuous run through `pts`, from rest to rest. The plan
        (plan_profile()) gives the tool speed along it; every tick each
        moving axis is sent its share of that speed, aimed at the end of its
        current one-direction stretch, so it never brakes early and lands
        exactly where that stretch ends. Corners get rounded by the axes'
        finite acceleration - by at most CORNER_TOL_CM, which the plan's
        corner speeds guarantee."""
        lim = {n: self._axis_limits(n) for n in ORDER}
        prof = plan_profile(pts, speed_cm_s,
                            [lim[n][1] for n in ORDER], [lim[n][2] for n in ORDER],
                            [lim[n][3] for n in ORDER])
        if prof is None:
            return                    # nothing to move
        self.log(f"# run of {len(prof.segs)} segment(s), {prof.total:.1f} s")
        last = {}                     # axis name -> (target steps, hz) last sent
        final = {n: self.cm_to_steps(n, pts[-1][i]) for i, n in enumerate(ORDER)}
        k_prev = 0                    # segment the previous tick was on
        t0 = time.time()
        while True:
            if self._stop.is_set():
                raise LinkError("stopped")
            if not self.connected:
                raise LinkError("link lost")
            if self.status["estop"] or any(self._axis(n)["state"] == "FAULT" for n in ORDER):
                self.stop()
                raise LinkError("controller faulted or e-stopped mid-path")
            el = time.time() - t0
            if progress:
                progress(prof.at(el)[1])
            # a command holds until the next tick, so send what the plan
            # needs halfway through it: the timing error is then half a
            # tick either way, which plan_profile() allows for at corners
            tc = el + TICK_S / 2
            k, _, v = prof.at(tc)
            if tc >= prof.total:
                k = len(prof.segs) - 1
            # feedback: how far each axis is off the plan, by the last status
            # line compared with where the plan had it when that was sent.
            # Open loop, the axes each lag their speed changes by their own
            # amount (z ramps far slower than x and y), and that difference
            # is what takes the tool off the line.
            off = [0.0, 0.0, 0.0]
            ts = self._status_t - t0
            if 0 < ts < prof.total and self.status_age() < 0.6:
                planned = prof.at(ts)[1]
                off = [planned[i] - self.steps_to_cm(n, self._axis(n)["pos"])
                       for i, n in enumerate(ORDER)]
            # every segment since the last tick: one shorter than a tick can
            # fall between two, and an axis that only moves in it must still
            # be sent where that motion ends
            for i, n in enumerate(ORDER):
                kk = next((j for j in range(k, k_prev - 1, -1)
                           if abs(prof.segs[j][3][i]) > 1e-12), None)
                if kk is None:
                    continue          # still: already aimed at where it stops
                u = prof.segs[kk][3][i]
                spc = lim[n][0]
                target = self.cm_to_steps(n, prof.run_end[kk][i])
                if kk == k and tc < prof.total:
                    # its share of the planned speed, corrected toward the plan
                    # (never reversed: it only ever heads for its stretch end)
                    # the axis reaches each new speed only after ramping to it
                    # at its full accel, which lags the plan by about
                    # (planned accel / full accel) * half a tick - so take the
                    # speed from that much further along the plan
                    acc_i = (prof.at(tc + TICK_S / 2)[2] - prof.at(tc - TICK_S / 2)[2])                         / TICK_S * abs(u)
                    lead = min(1.0, abs(acc_i) / lim[n][3]) * TICK_S / 2
                    speed = max(0.0, abs(v * u) + acc_i * lead
                                + math.copysign(1, u) * TRACK_GAIN * off[i])
                else:
                    speed = abs(prof.segs[kk][5] * u)     # catching up a missed stretch
                hz = max(1, round(max(speed, MIN_AXIS_SPEED_CM_S) * spc))
                acc = max(1, round(lim[n][3] * spc))
                prev = last.get(n)
                if prev is None or prev[0] != target or (
                        kk == k and tc < prof.total and hz != prev[1]):
                    self.send(f"MS {self.map[n].axis} {target} {hz} {acc}")
                    last[n] = (target, hz)
            k_prev = k
            if el >= prof.total:
                # Do NOT poll with "?": every status line is ~1.2 KB that the
                # firmware also writes to the USB serial port, which at 115200
                # baud blocks its main loop for ~100 ms each. The 250 ms stream
                # is enough to see arrival.
                arrived = all(abs(self._axis(n)["pos"] - final[n]) <= 2
                              and not self._axis(n)["moving"] for n in ORDER)
                if arrived and self.status_age() < 0.6 and el > prof.total + 0.3:
                    return
                if el > prof.total * 2 + 3:
                    raise LinkError("did not reach (%.1f, %.1f, %.1f); at (%.1f, %.1f, %.1f)"
                                    % (*pts[-1], *self.position_cm()))
            time.sleep(TICK_S)


class Profile:
    """A planned continuous run (plan_profile()): the tool's speed along a
    polyline, accelerating from rest, slowing for each corner and stopping at
    the end. segs[k] = (start point, length, start time, unit direction,
    speed in, peak speed, speed out, accel); run_end[k][i] is where axis i's
    current one-direction stretch ends, for a tool on segment k."""

    def __init__(self, segs, run_end, total):
        self.segs, self.run_end, self.total = segs, run_end, total

    def at(self, t):
        """(segment index, tool point, tool speed) at time t."""
        t = min(max(t, 0.0), self.total)
        for k, (a, length, t_start, u, v0, peak, v1, acc) in enumerate(self.segs):
            t_acc = (peak - v0) / acc
            d_acc = (peak * peak - v0 * v0) / (2 * acc)
            d_dec = (peak * peak - v1 * v1) / (2 * acc)
            t_cr = (length - d_acc - d_dec) / peak if peak > 0 else 0.0
            t_dec = (peak - v1) / acc
            dt = t - t_start
            if dt <= t_acc + t_cr + t_dec + 1e-9 or k == len(self.segs) - 1:
                if dt < t_acc:
                    d, v = v0 * dt + 0.5 * acc * dt * dt, v0 + acc * dt
                elif dt < t_acc + t_cr:
                    d, v = d_acc + peak * (dt - t_acc), peak
                else:
                    r = min(dt - t_acc - t_cr, t_dec)
                    d, v = d_acc + (length - d_acc - d_dec) + peak * r - 0.5 * acc * r * r, peak - acc * r
                d = min(max(d, 0.0), length)
                return k, tuple(a[i] + u[i] * d for i in range(3)), max(v, 0.0)
        a, length, _, u = self.segs[-1][:4]
        return len(self.segs) - 1, tuple(a[i] + u[i] * length for i in range(3)), 0.0


def plan_profile(pts, speed, axis_vmax, axis_amax, axis_afull, tol=None, lag=TICK_S / 2):
    """Plan a continuous run through the polyline `pts`, from rest to rest.

    speed: wanted tool speed, cm/s. axis_vmax / axis_amax: per axis (x, y,
    z), the most speed and acceleration the plan may ask of it; axis_afull:
    the acceleration the axis actually ramps at (its full firmware value).
    Each segment's cruise speed and acceleration are capped so no axis is
    asked for more than its share allows. At a corner, axis i's velocity
    has to jump by v * |change in direction_i|. That jump reaches the axis
    up to `lag` seconds off (the streaming tick), then the axis catches up
    with only the acceleration the plan leaves spare (axis_afull -
    axis_amax), so it strays off the line by up to
    jump * lag + jump^2 / (2 * spare); the corner speed is capped to keep
    that within `tol` (CORNER_TOL_CM).
    Then the usual forward/backward passes make every speed change
    reachable within its segment. Returns a Profile, or None if there is
    nothing to move."""
    tol = CORNER_TOL_CM if tol is None else tol
    segs = []
    for a, b in zip(pts, pts[1:]):
        length = math.dist(a, b)
        if length > 1e-9:
            segs.append((a, length, tuple((b[i] - a[i]) / length for i in range(3))))
    if not segs:
        return None
    vmax, amax = [], []
    for _, _, u in segs:
        vmax.append(min([speed] + [axis_vmax[i] / abs(u[i]) for i in range(3) if abs(u[i]) > 1e-12]))
        amax.append(min(axis_amax[i] / abs(u[i]) for i in range(3) if abs(u[i]) > 1e-12))
    n = len(segs)
    v = [0.0] * (n + 1)               # speed at each vertex
    for k in range(1, n):
        cap = min(vmax[k - 1], vmax[k])
        for i in range(3):
            du = abs(segs[k][2][i] - segs[k - 1][2][i])
            if du > 1e-9:
                a = max(axis_afull[i] - axis_amax[i], 1e-6)
                jump = a * (math.sqrt(lag * lag + 2 * tol / a) - lag)
                cap = min(cap, jump / du)
        v[k] = cap
    for k in range(n - 1, -1, -1):    # can it brake in time for what follows?
        v[k] = min(v[k], math.sqrt(v[k + 1] ** 2 + 2 * amax[k] * segs[k][1]))
    for k in range(n):                # can it get up to speed from what came before?
        v[k + 1] = min(v[k + 1], math.sqrt(v[k] ** 2 + 2 * amax[k] * segs[k][1]))

    out, t = [], 0.0
    for k, (a, length, u) in enumerate(segs):
        v0, v1, acc = v[k], v[k + 1], amax[k]
        peak = min(vmax[k], math.sqrt((2 * acc * length + v0 * v0 + v1 * v1) / 2))
        peak = max(peak, v0, v1)
        out.append((a, length, t, u, v0, peak, v1, acc))
        d_acc = (peak * peak - v0 * v0) / (2 * acc)
        d_dec = (peak * peak - v1 * v1) / (2 * acc)
        t += (peak - v0) / acc + (peak - v1) / acc
        if peak > 0:
            t += max(0.0, length - d_acc - d_dec) / peak

    # where each axis's one-direction stretch ends, seen from each segment
    run_end = []
    for k in range(n):
        ends = []
        for i in range(3):
            sgn = math.copysign(1, segs[k][2][i]) if abs(segs[k][2][i]) > 1e-12 else 0
            j = k
            while j + 1 < n and sgn != 0 and abs(segs[j + 1][2][i]) > 1e-12 \
                    and math.copysign(1, segs[j + 1][2][i]) == sgn:
                j += 1
            ends.append(segs[j][0][i] + segs[j][2][i] * segs[j][1])
        run_end.append(ends)
    return Profile(out, run_end, t)

"""Drive the esp32_4axis controller along a path planned by astar_core.

The firmware speaks one line-based protocol, over USB serial and over the WiFi
WebSocket on port 81 (this module uses the WebSocket, like stepper_gui.html; see
esp32_4axis/README.md): `M <axis> <steps>` moves an axis to an absolute step
position, and a JSON status line arrives every 250 ms. Each axis runs on its
own, so plain `M` commands to a far waypoint would NOT travel the straight line
A* planned - whichever axis is quickest would arrive first and the tool would
cut across a box. Instead each straight segment is sent with the firmware's
`MS <axis> <pos> <hz> <acc>` command (one move at a given speed and
acceleration): every axis gets its share of the tool speed and of the tool
acceleration, so all axes ramp up, cruise and brake together and stay on the
line. The tool comes to rest at each corner of the (smoothed) path.

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
ACCEL_FRACTION = 0.7      # of each axis's configured `acc`
SETTLE_TOL_CM = 0.3       # how close counts as "arrived"
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

    def follow(self, path, speed_cm_s=DEFAULT_SPEED_CM_S, progress=None):
        """Run the tool along `path` (list of (x, y, z) cm), one straight
        segment at a time. Blocks until it arrives, is stopped, or fails.
        `progress(point)` gets the measured tool position about 20 times a
        second. Raises LinkError if the machine is not ready or the path is
        unusable."""
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
        self.log(f"# following {len(path)} points, {len(path) - 1} segments")
        for a, b in zip(path, path[1:]):
            self._segment(a, b, speed_cm_s, progress)

    def _segment(self, a, b, speed_cm_s, progress):
        """One straight run a -> b as a single trapezoid on every axis. Each
        axis gets its share of the tool speed as its cruise speed and the same
        share of the tool acceleration as its acceleration (MS command), so all
        axes ramp, cruise and stop together and the tool stays on the line."""
        length = math.dist(a, b)
        if length < 1e-9:
            return
        self.log(f"# segment to ({b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f})")
        moves = []                    # (axis name, firmware axis, target steps, share)
        for i, n in enumerate(ORDER):
            target = self.cm_to_steps(n, b[i])
            share = abs(b[i] - a[i]) / length
            if target != self.cm_to_steps(n, a[i]):
                moves.append((n, self.map[n].axis, target, share))
        if not moves:
            return

        # tool speed / accel such that no axis exceeds its limit for its share
        v, acc = speed_cm_s, math.inf
        for n, _, _, share in moves:
            c = self._axis(n)["c"]
            steps_per_cm = c["spm"] / 100.0
            v = min(v, SPEED_FRACTION * c["run"] / (steps_per_cm * share))
            acc = min(acc, ACCEL_FRACTION * c["acc"] / (steps_per_cm * share))
        ramp = v * v / acc            # distance to accelerate and to brake
        if ramp > length:             # too short to reach v: triangle profile
            v = math.sqrt(acc * length)
        expected = length / v + v / acc

        for n, axis, target, share in moves:
            steps_per_cm = self._spm(n) / 100.0
            hz = max(1, round(v * share * steps_per_cm))
            a_steps = max(1, round(acc * share * steps_per_cm))
            self.send(f"MS {axis} {target} {hz} {a_steps}")
            self.log(f"> MS {axis} {target} {hz} {a_steps}")

        # Do NOT poll with "?": every status line is ~1.2 KB that the firmware
        # also writes to the USB serial port, which at 115200 baud blocks its
        # main loop for ~100 ms each. Polling at 20 Hz starves the WebSocket
        # heartbeat and the connection drops. The 250 ms stream is enough to
        # detect arrival; the display is dead-reckoned from the motion profile.
        t0 = time.time()
        while True:
            if self._stop.is_set():
                raise LinkError("stopped")
            if not self.connected:
                raise LinkError("link lost")
            if self.status["estop"] or any(self._axis(n)["state"] == "FAULT" for n in ORDER):
                self.stop()
                raise LinkError("controller faulted or e-stopped mid-path")
            time.sleep(0.05)
            el = time.time() - t0
            if progress:
                d = _profile_distance(el, length, v, acc)
                f = d / length
                progress(tuple(p + (q - p) * f for p, q in zip(a, b)))
            # status is at most 250 ms old, so only trust it once the move
            # should be over
            arrived = el > expected and all(
                abs(self._axis(n)["pos"] - target) <= 2 and not self._axis(n)["moving"]
                for n, _, target, _ in moves)
            if arrived and self.status_age() < 0.6:
                return
            if el > expected * 2 + 3:
                raise LinkError("did not reach (%.1f, %.1f, %.1f); at (%.1f, %.1f, %.1f)"
                                % (*b, *self.position_cm()))


def _profile_distance(t, length, v, acc):
    """Distance covered t seconds into a trapezoid (or triangle) move."""
    ta = v / acc                                # time to reach cruise speed
    cruise = max(0.0, length - v * ta)          # distance at constant v
    total = 2 * ta + cruise / v
    t = min(max(t, 0.0), total)
    if t < ta:
        return 0.5 * acc * t * t
    if t < ta + cruise / v:
        return 0.5 * acc * ta * ta + v * (t - ta)
    r = total - t
    return length - 0.5 * acc * r * r

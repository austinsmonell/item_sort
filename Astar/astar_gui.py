"""GUI for the 3D A* pathfinder, built with tkinter (Python standard library
- nothing to install). It plans through the environment defined in
arena_env.py (walls, boxes, carriage and lift) using plan_path() from
astar_core.py - the GUI runs the real algorithm, it does not reimplement it.

Left: a top-down map (x right, y up, x = 0 y = 0 at the bottom left of the
moveable area). Start is always wherever the mechanism actually is - read
from the machine once connected, otherwise its last homed/commanded pose -
so there is nothing to set for it. Click the map to place the Goal; on a box
it aims at that box's lift point, otherwise it goes at the lift reading set
by the Lift z slider. The carriage and lift plate are drawn at their current
pose and turn red if they overlap anything. Boxes show their top height in
cm.

Run A* plans the tool point (carriage x, carriage y, lift z) around every box,
allowing for the size of the carriage and plate. The Path slider then steps
the carriage and lift along the planned path. "Send path" runs it on the
machine, or - when not connected - simulates it, moving Start to its end.

Picking up and placing a box: once a path that ends with a box's full pick
sequence has run, that box is on the lift - it moves with the tool point
and A* plans around the walls, floor and other boxes for it too. Drag it
on the map to where it should go (it rests on the floor, or on top of
whatever box is under it) and Run A*: the path carries it there, lowers it
into place, and runs the pick sequence in reverse to drop it, after which
it is an ordinary box again.

Right: a 3D view of the whole arena. Drag to rotate, mouse wheel to zoom,
click to set the Goal the same way as on the map - near a box's yellow lift
marker it aims at that box, otherwise at the floor under the cursor.

Change the arena (dimensions, boxes) in arena_env.py.

Run with:  python astar_gui.py
"""

import math
import queue
import threading
import time
import traceback
import tkinter as tk

from arena_env import (BOX_TYPES, HIT_COLOR, KIND_COLOR, Arena, Solid,
                       make_boxes)
from astar_core import (PLAN_TIMEOUT_S, RESOLUTION_CM, BoxObstacles, PlanningTimeout,
                        cm_to_node, plan_path, smooth_path)
from esp_link import DEFAULT_SPEED_CM_S, EspLink, LinkError

PX = 3                        # map pixels per cm
VIEW_W = 520
VIEW_H = 480
MAX_EXPLORED_DOTS = 4000
PATH_COLOR = "#46b478"
LIMIT_COLOR = "#4a7db5"       # the moveable area / lift limits (dashed)
PLACE_COLOR = "#9fd0ff"       # where a carried box is to be placed
DEFAULT_PICK_SEQUENCE = "y-2, z4"  # starting pick sequence for every box type


def snap(v, lo, hi):
    """Round a cm value to the resolution and keep it within lo..hi."""
    return min(max(round(v / RESOLUTION_CM) * RESOLUTION_CM, lo), hi)


_AXES = "xyz"


def parse_sequence(text):
    """Parse a pick instruction sequence such as "y1, z-0.5, x0.2" into a
    list of (axis index 0/1/2, signed delta cm) steps, run in order, each a
    tool-point move relative to wherever the previous step (or the box's
    exact lift point) left off. Raises ValueError, with a message fit to
    show the user, on anything that doesn't parse."""
    steps = []
    for tok in text.split(","):
        tok = tok.strip()
        if not tok:
            continue
        axis, num = tok[0].lower(), tok[1:]
        if axis not in _AXES:
            raise ValueError(f"'{tok}': expected to start with x, y, or z")
        try:
            delta = float(num)
        except ValueError:
            raise ValueError(f"'{tok}': '{num}' isn't a number")
        steps.append((_AXES.index(axis), delta))
    return steps


def reverse_sequence(steps):
    """The sequence of moves that undoes `steps`: reverse order, each delta
    negated. The drop sequence for a box type is always this, run on the
    pick sequence - there is no separate drop sequence to configure."""
    return [(axis, -delta) for axis, delta in reversed(steps)]


class AStarGui:
    def __init__(self, root, arena=None):
        self.root = root
        self.arena = arena or Arena(obstacles=make_boxes())
        d = self.arena.dims
        root.title("3D A* Pathfinding")

        # map geometry: the outside of the walls fills the canvas
        ix0, iy0, ix1, iy1 = self.arena.inner_box()
        t = d.wall_thickness
        self.wx0, self.wy1 = ix0 - t, iy1 + t
        self.map_w = int((ix1 - ix0 + 2 * t) * PX)
        self.map_h = int((iy1 - iy0 + 2 * t) * PX)

        self.z = tk.DoubleVar(value=d.move_z)
        self.diagonals = tk.BooleanVar(value=True)
        self.show_explored = tk.BooleanVar(value=False)
        self.cursor = tk.StringVar(value="")

        # Start is always the mechanism's actual pose (x, y, z tool point,
        # cm): synced from the machine while connected, otherwise wherever
        # it was last homed/commanded to. There is no manual "set Start"
        # (see the arena.home() call below, which sets its initial value).
        self.start = None
        self.goal = None
        self.target_box = None    # the Solid set_goal() last aimed at, if any
        self.final_point = None   # the exact target set_goal() backed off from, if any
        self.final_sequence = []  # pick steps to run from final_point, if any

        # the box the plate is currently engaged with (touching/overlapping
        # per its pick sequence), and how much of that sequence actually
        # ran - set by append_final_contact(), read by disengage_start() to
        # reverse exactly that before planning a move away from it.
        self.engaged_box = None
        self.engaged_sequence = []
        self.engaged_steps = 0

        # carrying a box (self.arena.carried): the pick steps that picked it
        # up, reversed to drop it, and where it was picked from (Clear puts
        # it back there when not connected).
        self.carry_sequence = []
        self.carry_origin = None
        # a place goal for the carried box: the box where it is to end up
        # (also drawn as the drag ghost), and the grab offset while dragging
        self.place_box = None
        self._drag = None

        self.path = []
        self.path_ignore = []
        self.on_arrival = None    # applies what the path does (pick, drop) once it has run
        self.path_rule = None     # see show_pose()
        self.explored = set()

        self.yaw = math.radians(35)
        self.pitch = math.radians(55)
        self.zoom = 2.2          # 3D view pixels per cm
        self._view_drag = None
        self._view_press = None

        toolbar = tk.Frame(root)
        toolbar.pack(fill="x", padx=8, pady=8)
        tk.Label(toolbar, text="Click the map to set the Goal:").pack(side="left")
        tk.Checkbutton(toolbar, text="Diagonals", variable=self.diagonals
                       ).pack(side="left", padx=(12, 2))
        tk.Checkbutton(toolbar, text="Show explored (3D)",
                       variable=self.show_explored).pack(side="left", padx=2)
        tk.Label(toolbar, text="Clearance (cm):").pack(side="left", padx=(12, 0))
        self.clearance = tk.DoubleVar(value=0.5)
        tk.Spinbox(toolbar, from_=0, to=5, increment=0.25, width=5,
                   textvariable=self.clearance).pack(side="left", padx=2)
        tk.Button(toolbar, text="Run A*", command=self.run).pack(side="left", padx=(12, 2))
        tk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=2)
        tk.Button(toolbar, text="Place box here", command=self.place_here
                  ).pack(side="left", padx=2)

        # per box type, the sequence of tool-point moves to run once the
        # plate reaches that box's exact lift point, to pick it up (see
        # append_final_contact and sequence_for()). Steps may touch/overlap
        # that box - see in_collision()'s ignore - since that's the point of
        # engaging it; every other obstacle still blocks them. There's no
        # separate drop sequence to configure: leaving an engaged box always
        # runs this same sequence in reverse to disengage (disengage_start()).
        self.pick_seq = {kind: tk.StringVar(value=DEFAULT_PICK_SEQUENCE) for kind in BOX_TYPES}

        seqframe = tk.LabelFrame(
            root, text='Pick sequence per box type, run once the plate reaches '
                       'the box - e.g. "y1, z-0.5" (axis + signed cm, in order). '
                       "Once it has run the box is on the lift; click or drag on the map "
                       "to place it, which reverses the sequence to drop it.")
        seqframe.pack(fill="x", padx=8, pady=(0, 8))
        tk.Label(seqframe, text="Box type").grid(row=0, column=0, padx=(4, 4))
        tk.Label(seqframe, text="Pick sequence").grid(row=0, column=1, padx=4)
        tk.Label(seqframe, text="Lift height (cm above box bottom)").grid(row=0, column=2, padx=4)
        # per box type, how far above its bottom the plate lifts it from
        self.lift_height = {kind: tk.StringVar(value=f"{self.arena.lift_height(kind):g}")
                            for kind in BOX_TYPES}
        for r, kind in enumerate(BOX_TYPES, start=1):
            tk.Label(seqframe, text=kind).grid(row=r, column=0, sticky="w", padx=(4, 4))
            tk.Entry(seqframe, textvariable=self.pick_seq[kind], width=20
                     ).grid(row=r, column=1, padx=4, pady=1, sticky="w")
            tk.Spinbox(seqframe, from_=0, to=50, increment=0.25, width=6,
                       textvariable=self.lift_height[kind]
                       ).grid(row=r, column=2, padx=4, pady=1, sticky="w")
            self.lift_height[kind].trace_add(
                "write", lambda *_, k=kind: self.on_lift_height(k))

        boxbar = tk.Frame(root)
        boxbar.pack(fill="x", padx=8, pady=(0, 6))
        tk.Label(boxbar, text="Boxes:").pack(side="left")
        self.load_kind = tk.StringVar(value=next(iter(BOX_TYPES)))
        tk.OptionMenu(boxbar, self.load_kind, *BOX_TYPES).pack(side="left", padx=(8, 2))
        tk.Button(boxbar, text="Load on lift", command=self.load_on_lift
                  ).pack(side="left", padx=2)
        tk.Button(boxbar, text="Remove all boxes", command=self.remove_all_boxes
                  ).pack(side="left", padx=(12, 2))

        zbar = tk.Frame(root)
        zbar.pack(fill="x", padx=8)
        tk.Label(zbar, text="Lift z (cm):").pack(side="left")
        tk.Scale(zbar, from_=0, to=d.move_z, resolution=RESOLUTION_CM,
                 orient="horizontal", length=240, variable=self.z
                 ).pack(side="left", padx=4)
        tk.Label(zbar, text="Path:").pack(side="left", padx=(12, 0))
        self.scrub = tk.Scale(zbar, from_=0, to=0, orient="horizontal",
                              length=240, showvalue=False,
                              command=self.on_scrub, state="disabled")
        self.scrub.pack(side="left", padx=4)
        tk.Label(zbar, textvariable=self.cursor, width=22, anchor="w"
                 ).pack(side="left", padx=8)

        # live readout of the mechanism at the pose shown (update_readout())
        self.readout = tk.StringVar(value="")
        tk.Label(root, textvariable=self.readout, anchor="w", font=("TkFixedFont", 9)
                 ).pack(fill="x", padx=8, pady=(4, 0))

        mbar = tk.Frame(root)
        mbar.pack(fill="x", padx=8, pady=(6, 0))
        tk.Label(mbar, text="Machine host:").pack(side="left")
        self.port = tk.StringVar(value="stepper.local")
        tk.Entry(mbar, textvariable=self.port, width=18).pack(side="left", padx=4)
        self.connect_btn = tk.Button(mbar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=2)
        tk.Label(mbar, text="Speed (cm/s):").pack(side="left", padx=(12, 0))
        self.speed = tk.DoubleVar(value=DEFAULT_SPEED_CM_S)
        tk.Spinbox(mbar, from_=0.5, to=100, increment=0.5, width=5,
                   textvariable=self.speed).pack(side="left", padx=4)
        self.send_btn = tk.Button(mbar, text="Send path (simulate if not connected)",
                                  command=self.send_path)
        self.send_btn.pack(side="left", padx=(12, 2))
        tk.Button(mbar, text="STOP", fg="white", bg="#c03030",
                  command=self.stop_machine).pack(side="left", padx=2)

        views = tk.Frame(root)
        views.pack(padx=8)
        self.canvas = tk.Canvas(views, width=self.map_w, height=self.map_h,
                                background="#222222")
        self.canvas.pack(side="left")
        self.view = tk.Canvas(views, width=VIEW_W, height=VIEW_H, background="#161616")
        self.view.pack(side="left", padx=(8, 0))

        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.on_motion)
        self.view.bind("<ButtonPress-1>", self.on_view_press)
        self.view.bind("<B1-Motion>", self.on_view_drag)
        self.view.bind("<ButtonRelease-1>", self.on_view_release)
        self.view.bind("<MouseWheel>",
                       lambda e: self.zoom_by(1.1 if e.delta > 0 else 1 / 1.1))
        self.view.bind("<Button-4>", lambda e: self.zoom_by(1.1))
        self.view.bind("<Button-5>", lambda e: self.zoom_by(1 / 1.1))

        self.status = tk.StringVar(
            value=f"Moveable area {d.move_x:g} x {d.move_y:g} cm, lift z 0 to "
                  f"{d.move_z:g}, {RESOLUTION_CM:g} cm resolution. Start is "
                  "the mechanism's actual pose. Click the map to set a Goal, "
                  "then Run A*. While a box is on the lift, clicking or "
                  "dragging on the map places it.")
        tk.Label(root, textvariable=self.status, anchor="w", justify="left",
                 wraplength=1060).pack(fill="x", padx=8, pady=8)

        self.events = queue.Queue()   # worker thread -> tk main thread
        self.link = EspLink(log=lambda m: self.events.put(("log", m)))
        self.running = False
        self.root.after(50, self.pump)

        tk.Label(root, text="Controller messages:", anchor="w").pack(fill="x", padx=8)
        self.logbox = tk.Text(root, height=6, state="disabled", background="#161616",
                              foreground="#bbbbbb")
        self.logbox.pack(fill="x", padx=8, pady=(0, 8))

        self.arena.home()
        self.start = tuple(self.arena.pos)
        self.draw()

    # ---- machine ----------------------------------------------------------

    def pump(self):
        """Apply what the serial/worker threads posted (tk is not thread safe)."""
        moved = False
        try:
            while True:
                kind, val = self.events.get_nowait()
                if kind == "pose":
                    self.show_pose(*val)
                    moved = True
                elif kind == "done":
                    msg, arrived = val
                    self.running = False
                    self.send_btn.config(state="normal")
                    if arrived is not None:
                        self.arrive(*arrived)
                    self.status.set(msg)
                else:                 # a "#" line from the firmware
                    self.logbox.config(state="normal")
                    self.logbox.insert("end", val + "\n")
                    self.logbox.see("end")
                    self.logbox.config(state="disabled")
        except queue.Empty:
            pass
        if self.sync_start():
            moved = True
        if moved:
            self.draw()
        self.root.after(50, self.pump)

    def toggle_connect(self):
        if self.link.connected:
            self.link.close()
            self.connect_btn.config(text="Connect")
            self.status.set("Disconnected.")
            return
        try:
            self.link.connect(self.port.get().strip())
        except LinkError as e:
            self.status.set(f"Connect failed: {e}")
            return
        self.connect_btn.config(text="Disconnect")
        bad = self.link.ready_problems()
        self.status.set("Connected." + (" Not ready: " + "; ".join(bad) if bad else
                                        " Drives on. Make sure every axis is homed or zeroed (Z) first."))

    def sync_start(self):
        """While connected, Start IS the machine: the carriage and lift where
        they are now. Returns True if Start moved."""
        if not self.link.connected or self.running or self.link.status is None:
            return False
        d = self.arena.dims
        try:
            pos = self.link.position_cm()
        except (LinkError, KeyError):
            return False
        new = tuple(snap(v, 0.0, hi) for v, hi in zip(pos, (d.move_x, d.move_y, d.move_z)))
        if new == self.start:
            return False
        self.start = new
        self.arena.move_to(*new)
        return True

    def send_path(self):
        if self.running:
            return
        if not self.path:
            self.status.set("Run A* first - there is no path to send.")
            return
        # captured now: clicking the map while it runs clears these
        path, speed, rule = list(self.path), self.speed.get(), self.path_rule
        arrived = (self.on_arrival, self.path[-1])
        self.running = True
        self.send_btn.config(state="disabled")
        if not self.link.connected:
            self.simulate(path, speed, rule, arrived)
            return
        self.status.set("Following the path... (STOP aborts)")
        # followed in two parts, split where the pick/drop sequence starts,
        # so the box can be shown held still through it (show_pose())
        r = rule[0] if rule else len(path) - 1

        def work():
            try:
                for part, k in ((path[:r + 1], None), (path[r:], r)):
                    if len(part) >= 2:
                        self.link.follow(part, speed, progress=lambda p, k=k:
                                         self.events.put(("pose", (p, rule, k))))
                self.events.put(("done", ("Arrived at the goal.", arrived)))
            except LinkError as e:
                self.events.put(("done", (f"Path not completed: {e}", None)))
            except Exception:         # a bug must not leave the GUI "following" forever
                self.events.put(("log", traceback.format_exc()))
                self.events.put(("done", ("Path aborted by an internal error - see the "
                                          "controller messages below.", None)))
                self.link.stop()
        threading.Thread(target=work, daemon=True).start()

    def simulate(self, path, speed, rule, arrived):
        """Not connected: play the path back on screen at `speed`, then treat
        it as run (arrive()), the same as the machine finishing it."""
        step = max(speed, 0.1) * 0.05          # cm per 50 ms frame
        poses = []
        for j, (a, b) in enumerate(zip(path, path[1:]), start=1):
            n = max(1, math.ceil(math.dist(a, b) / step))
            poses += [(tuple(a[i] + (b[i] - a[i]) * k / n for i in range(3)), j)
                      for k in range(1, n + 1)]
        frames = iter(poses)
        self.status.set("Not connected - simulating the path... (STOP aborts)")

        def tick():
            if not self.running:               # STOP
                return
            f = next(frames, None)
            if f is None:
                self.events.put(("done", ("Simulated path finished.", arrived)))
                return
            self.show_pose(f[0], rule, f[1])
            self.draw()
            self.root.after(50, tick)
        tick()

    def arrive(self, on_arrival, end):
        """A path has been run to its end: apply what it did (picked up or
        dropped a box, see run()), and - when not connected, so nothing
        else reports the pose - make its end the new Start. The path and
        Goal are used up."""
        if not self.link.connected:
            self.start = end
        self.arena.box_override = None
        if on_arrival is not None:
            on_arrival()
        self.reset_goal()
        self.arena.move_to(*self.start)
        self.draw()

    def stop_machine(self):
        if self.running and not self.link.connected:   # stop the simulation
            self.running = False
            self.send_btn.config(state="normal")
            self.arena.box_override = None
            d = self.arena.dims
            # on the lattice, like every other Start (plan_path snaps to it)
            self.start = tuple(snap(v, 0.0, hi) for v, hi in
                               zip(self.arena.pos, (d.move_x, d.move_y, d.move_z)))
            self.status.set("Simulation stopped - Start is where it stopped. A pick "
                            "or drop it didn't finish has not happened.")
            self.draw()
            return
        if self.link.connected:
            try:
                self.link.stop()
            except LinkError:
                pass
            self.status.set("STOP sent.")

    # ---- coordinates ----------------------------------------------------

    def sx(self, x):
        return (x - self.wx0) * PX

    def sy(self, y):
        return (self.wy1 - y) * PX

    def cm_at(self, event):
        d = self.arena.dims
        x = event.x / PX + self.wx0
        y = self.wy1 - event.y / PX
        return snap(x, 0.0, d.move_x), snap(y, 0.0, d.move_y)

    def raw_cm_at(self, event):
        """Map pixel -> (x, y) cm, not snapped or limited to the moveable
        area (a carried box can sit outside it)."""
        return event.x / PX + self.wx0, self.wy1 - event.y / PX

    # ---- pose helpers ---------------------------------------------------

    def in_collision(self, point, ignore=None):
        """True if the mechanism collides with anything at `point`. If
        `ignore` is a solid's name, a collision with only that solid
        doesn't count - used for pick/drop sequence steps, which are
        expected to touch or overlap the one box they're engaging."""
        saved = list(self.arena.pos)
        self.arena.move_to(*point)
        hits = self.arena.collisions()
        if ignore is not None:
            hits = [h for h in hits if ignore not in h]
        hit = bool(hits)
        self.arena.pos[:] = saved
        return hit

    def show_pose(self, p, rule=None, k=None):
        """Put the mechanism at `p`, the pose on the way to path point k.
        `rule` is (first path index, box name, box) for a path ending in a
        pick or drop sequence: from that index on, the box sits still at
        `box` - it isn't on the lift until the pick sequence has finished,
        and is off it as soon as it has been set down."""
        self.arena.box_override = (rule[1:] if rule and k is not None and k >= rule[0]
                                   else None)
        self.arena.move_to(*p)

    def invalidate(self):
        self.arena.box_override = None
        self.path_rule = None
        self.path = []
        self.path_ignore = []  # parallel to self.path: a solid name to excuse
                                # from that point's collision check, or None
        self.on_arrival = None
        self.explored = set()
        self.scrub.config(state="disabled", to=0)
        self.scrub.set(0)

    # ---- editing --------------------------------------------------------

    def on_motion(self, event):
        x, y = self.cm_at(event)
        self.cursor.set(f"x {x:g}  y {y:g} cm")

    def box_offset_point(self, box, offset):
        """The tool point that puts the plate's lift point `offset` cm out
        from `box`'s lift point (`box` a Solid: its type sets the lift
        height), along the box's back face normal (+y):
        offset 0 is the plate touching the box, positive backs off from it."""
        d = self.arena.dims
        lx, ly, lz = self.arena.box_lift_point(box)
        tx, ty, tz = self.arena.tool_point_for_lift((lx, ly + offset, lz))
        return (snap(tx, 0.0, d.move_x), snap(ty, 0.0, d.move_y), snap(tz, 0.0, d.move_z))

    def box_exact_point(self, box):
        """The tool point that puts the plate's lift point exactly on
        `box`'s lift point (the plate touching the box). The first stop of
        a box goal; any configured pick/drop sequence (see sequence_for())
        runs from here."""
        return self.box_offset_point(box, 0.0)

    def box_goal_point(self, box):
        """The tool point that puts the plate's lift point on `box`'s lift
        point, backed off from the box's back face by a bit more than the
        clearance setting.

        A* plans in the box's *grown* configuration space (grown by the
        clearance setting, so the path stays that far off every obstacle),
        so a goal placed exactly touching the box - zero clearance - sits
        just inside that grown region and is unreachable. Backing off keeps
        the goal just outside it, so a path can actually reach it; run()
        then closes that last bit itself, once the fine path is smoothed,
        with append_final_contact().
        """
        try:
            clearance = max(0.0, self.clearance.get())
        except tk.TclError:
            clearance = 0.5
        return self.box_offset_point(box, clearance + RESOLUTION_CM)

    def sequence_for(self, kind):
        """The parsed pick sequence configured for box type `kind`, or []
        if there's none, the type is unknown (a plain "obstacle" box rather
        than a BOX_TYPES kind), or the text doesn't parse - in which case
        the status bar explains why."""
        var = self.pick_seq.get(kind)
        if var is None:
            return []
        try:
            return parse_sequence(var.get())
        except ValueError as e:
            self.status.set(f"Bad pick sequence for '{kind}': {e}")
            return []

    def goal_for_box(self, x, y):
        """If (x, y) lands on a box's footprint, the box and its goal point
        from box_goal_point(); otherwise None."""
        for o in self.arena.obstacles:
            b = o.box
            if b[0] <= x <= b[3] and b[1] <= y <= b[4]:
                return o, self.box_goal_point(o)
        return None

    def set_goal(self, point, box=None, place=None):
        """Common tail for setting the Goal from either view: update state,
        check for collision there, and report status. `box`: a box to go
        and pick up. `place`: the tool point that sets the carried box down
        where set_place_goal() worked out (`point` is then the approach
        just above it)."""
        if box is not None and self.arena.carried is not None:
            self.status.set(f"Place {self.arena.carried[0].name} (drag it on the "
                            "map) before picking up another box.")
            return
        self.goal = point
        self.target_box = box
        if box is not None:
            self.final_point = self.box_exact_point(box)
            self.final_sequence = self.sequence_for(box.kind)
        elif place is not None:
            self.final_point = place
            self.final_sequence = []
        else:
            self.final_point = point
            self.final_sequence = []
        if place is None:
            self.place_box = None
        self.invalidate()
        self.arena.move_to(*point)  # show the mechanism where it was put
        hits = self.arena.collisions()
        if hits:
            self.status.set("In collision here: "
                            + "; ".join(f"{m} / {s}" for m, s in hits))
        elif box is not None:
            self.status.set(f"Goal set to lift {box.name} - carriage "
                            f"({point[0]:g}, {point[1]:g}), lift z {point[2]:g}.")
        elif place is not None:
            b = self.place_box
            self.status.set(f"Goal set to place {self.arena.carried[0].name} at "
                            f"({b[0]:g}, {b[1]:g}), resting at z {b[2]:g} - carriage "
                            f"({place[0]:g}, {place[1]:g}), lift z {place[2]:g}.")
        else:
            self.status.set(f"Goal set to ({point[0]:g}, {point[1]:g}, {point[2]:g}).")
        self.draw()

    def carried_place_box(self, x0, y0):
        """Where the carried box ends up if set down with its corner nearest
        the origin at (x0, y0): on the nearest surface below it - the top
        of the highest box under that footprint, or the floor."""
        off = self.arena.carried[1]
        x1, y1 = x0 + off[3] - off[0], y0 + off[4] - off[1]
        z0 = self.arena.support_height(x0, y0, x1, y1)
        return (x0, y0, z0, x1, y1, z0 + off[5] - off[2])

    def set_place_goal(self, x0, y0):
        """Aim to set the carried box down at carried_place_box(x0, y0).
        The Goal A* plans to is above that spot by a bit more than the
        clearance, since (like box_goal_point()) the resting place itself is
        inside the clearance-grown floor/support; run() lowers it the rest
        of the way and drops it (append_place())."""
        a, d = self.arena, self.arena.dims
        box = self.carried_place_box(x0, y0)
        self.place_box = box
        self.goal = None
        self.invalidate()
        clash = a.overlapping(box)
        if clash:
            self.status.set("Can't place it there - it would overlap " + ", ".join(clash) + ".")
            self.draw()
            return
        off = a.carried[1]
        place = tuple(box[i] - off[i] for i in range(3))   # box bottom on the surface
        if any(not -1e-9 <= v <= hi + 1e-9 for v, hi in zip(place, (d.move_x, d.move_y, d.move_z))):
            self.status.set("Can't place it there - out of the carriage's reach (it "
                            f"would need tool point {place[0]:g}, {place[1]:g}, "
                            f"{place[2]:g}).")
            self.draw()
            return
        try:
            clearance = max(0.0, self.clearance.get())
        except tk.TclError:
            clearance = 0.5
        above = (place[0], place[1],
                 min(max(place[2], box[2] - off[2] + clearance + RESOLUTION_CM), d.move_z))
        self.set_goal(above, place=place)

    def on_lift_height(self, kind):
        """A lift height was edited: use it for every box of that type from
        now on. A Goal aimed at a box is aimed afresh, since its lift point
        moved; a half-typed value that isn't a number is ignored."""
        try:
            h = float(self.lift_height[kind].get())
        except (ValueError, tk.TclError):
            return
        if h < 0 or h == self.arena.lift_height(kind):
            return
        self.arena.lift_heights[kind] = h
        if self.target_box is not None and self.target_box.kind == kind and not self.running:
            self.set_goal(self.box_goal_point(self.target_box), self.target_box)
        else:
            self.draw()

    def load_on_lift(self):
        """Put a new box of the chosen type on the lift, where a finished
        pick would have left it with the lift at Start: that type's pick
        sequence, undone from Start, gives the contact pose, where the
        plate's lift point is on the box's lift point. For a box that's
        already on the real lift, or to try a place without a pick first."""
        kind = self.load_kind.get()
        if self.running:
            self.status.set("A path is running - wait for it or press STOP first.")
            return
        if self.arena.carried is not None:
            self.status.set(f"{self.arena.carried[0].name} is already on the lift.")
            return
        self.sync_start()
        seq = self.sequence_for(kind)
        contact = list(self.start)
        for axis, delta in seq:
            contact[axis] -= delta
        lx, ly, lz = self.arena.plate_lift_point(*contact)
        w, dp, h = BOX_TYPES[kind]
        # on the lattice, like every box a real pick lifts (which is why the
        # plate's own lift point may sit a hair off the box's)
        x0, y1, z0 = (snap(v, -1e6, 1e6) for v in (lx - w / 2, ly, lz - self.arena.lift_height(kind)))
        taken = {o.name for o in self.arena.obstacles}
        n = 1
        while f"{kind} box {n}" in taken:
            n += 1
        box = Solid(f"{kind} box {n}", (x0, y1 - dp, z0, x0 + w, y1, z0 + h), kind)
        clash = self.arena.overlapping(box.box)
        if clash:
            self.status.set(f"No room for a {kind} box on the lift here - it would "
                            f"overlap {', '.join(clash)}. Move the lift first.")
            return
        self.reset_goal()
        self.engaged_box, self.engaged_sequence, self.engaged_steps = None, [], 0
        self.arena.carry(box, self.start)
        self.carry_sequence, self.carry_origin = seq, None
        self.arena.move_to(*self.start)
        self.status.set(f"{box.name} is on the lift - click or drag on the map to place it.")
        self.draw()

    def remove_all_boxes(self):
        """Empty the arena: every box, including one on the lift."""
        if self.running:
            self.status.set("A path is running - wait for it or press STOP first.")
            return
        self.reset_goal()
        self.engaged_box, self.engaged_sequence, self.engaged_steps = None, [], 0
        self.arena.obstacles.clear()
        self.arena.carried = None
        self.carry_sequence, self.carry_origin = [], None
        self.arena.move_to(*self.start)
        self.status.set("Removed all boxes.")
        self.draw()

    def place_here(self):
        """Give up on moving the carried box: set it down right where it is
        (on the surface under it) and plan that - lower it, then the drop
        sequence - ready to Send."""
        if self.arena.carried is None:
            self.status.set("No box on the lift to place.")
            return
        if self.running:
            self.status.set("The path is still running - press STOP first, then Place box here.")
            return
        self.sync_start()
        b = self.arena.carried_solid(*self.start).box
        self.set_place_goal(snap(b[0], -1e6, 1e6), snap(b[1], -1e6, 1e6))
        if self.goal is not None:
            self.run()

    def carried_footprint(self):
        """(x0, y0, x1, y1) to grab to drag the carried box: its place
        ghost if it has one, else the carried box itself."""
        b = self.place_box or self.arena.carried_solid(*self.arena.pos).box
        return b[0], b[1], b[3], b[4]

    def on_press(self, event):
        if self.running:
            return
        self.on_motion(event)
        self._drag = None
        if self.arena.carried is not None:
            # carrying a box, every click places it: grab the box (or its
            # place outline) to drag it, or click anywhere to put it there,
            # centred on the click (and keep dragging from there)
            rx, ry = self.raw_cm_at(event)
            x0, y0, x1, y1 = self.carried_footprint()
            if x0 <= rx <= x1 and y0 <= ry <= y1:
                self._drag = (x0 - rx, y0 - ry)   # grab offset to the box corner
            else:
                self._drag = ((x0 - x1) / 2, (y0 - y1) / 2)
                self.on_drag(event)
            return
        x, y = self.cm_at(event)
        box_hit = self.goal_for_box(x, y)
        if box_hit is not None:
            box, point = box_hit
        else:
            box, point = None, (x, y, snap(self.z.get(), 0.0, self.arena.dims.move_z))
        self.set_goal(point, box)

    def on_drag(self, event):
        if self.running:
            return
        if self._drag is None:
            self.on_press(event)
            return
        self.on_motion(event)
        rx, ry = self.raw_cm_at(event)
        self.place_box = self.carried_place_box(
            round((rx + self._drag[0]) / RESOLUTION_CM) * RESOLUTION_CM,
            round((ry + self._drag[1]) / RESOLUTION_CM) * RESOLUTION_CM)
        self.goal = None
        self.invalidate()
        self.draw_map()               # the ghost follows the mouse; 3D on release

    def on_release(self, event):
        if self._drag is None or self.running:
            return
        self._drag = None
        if self.place_box is not None:
            self.set_place_goal(self.place_box[0], self.place_box[1])

    def on_scrub(self, value):
        if self.path:
            i = int(float(value))
            self.show_pose(self.path[i], self.path_rule, i)
            self.draw()

    def run(self):
        self.arena.box_override = None   # plan with the boxes where they really are
        self.path_rule = None
        self.sync_start()
        self.infer_engagement()
        if self.goal is None:
            self.status.set("Set a Goal first.")
            return
        for name, p in (("Start", self.start), ("Goal", self.goal)):
            ignore = self.engaged_box.name if self.engaged_box else None
            if self.in_collision(p, ignore=ignore):
                saved = list(self.arena.pos)
                self.arena.move_to(*p)
                hits = sorted({h for pair in self.arena.collisions() if ignore not in pair
                               for h in pair[1:]})
                self.arena.pos[:] = saved
                self.status.set(f"{name} is in collision with {', '.join(hits)}, so no "
                                "path can start or end there. Move it.")
                return
        self.status.set("Planning...")
        self.root.update_idletasks()
        self.explored = set()
        t = time.time()
        try:
            clearance = max(0.0, self.clearance.get())
        except tk.TclError:
            clearance = 0.5
        plan_start, prepend_pts, prepend_ignore = self.plan_start_and_prepend(clearance)
        try:
            self.path = plan_path(
                self.arena, plan_start, self.goal, diagonals=self.diagonals.get(),
                explored=self.explored if self.show_explored.get() else None,
                clearance=clearance, timeout=PLAN_TIMEOUT_S)
        except PlanningTimeout:
            self.path = []
            self.status.set(f"Planning timed out after {PLAN_TIMEOUT_S:g} s without "
                            "finding a path - try a different Goal, or lower the "
                            "Clearance.")
            self.draw()
            return
        # the lattice staircase makes the motors wiggle: pull it into straight runs
        self.path = smooth_path(self.arena, self.path, clearance=clearance)
        if self.path:
            self.path_ignore = [None] * len(self.path)
            if prepend_pts:
                self.path = prepend_pts + self.path
                self.path_ignore = prepend_ignore + self.path_ignore
        sequence_note = ""
        if self.path and self.place_box is not None:
            sequence_note = self.finish_place_path()
        elif self.path:
            sequence_note = self.finish_pick_path()
        dt = time.time() - t
        if self.path:
            length = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
            bad = sum(1 for p, ig in zip(self.path, self.path_ignore)
                     if self.in_collision(p, ignore=ig))
            check = ("collision-free" if not bad
                     else f"WARNING: {bad} points collide")
            self.status.set(f"Path found: {len(self.path)} points, {length:.1f} cm, "
                            f"{check} ({dt:.1f} s). Drag the Path slider to step "
                            f"through it.{sequence_note}")
            self.scrub.config(state="normal", to=len(self.path) - 1)
            self.scrub.set(0)
            self.show_pose(self.path[0], self.path_rule, 0)
        else:
            self.status.set(f"No path found ({dt:.1f} s).")
        self.draw()

    def finish_pick_path(self):
        """Tail of run() for any Goal but a place: append the contact and
        pick sequence (append_final_contact()), and set self.on_arrival to
        what the path leaves engaged once it has run. A box whose whole
        pick sequence ran is then on the lift (Arena.attach()). Returns a
        note for the status bar."""
        box, seq = self.target_box, self.final_sequence
        reached_box, steps_done = self.append_final_contact()
        end = self.path[-1]
        on_top = [] if box is None else [
            o.name for o in self.arena.obstacles
            if abs(o.box[2] - box.box[5]) < 1e-9 and o.box[0] < box.box[3]
            and box.box[0] < o.box[3] and o.box[1] < box.box[4] and box.box[1] < o.box[4]]
        carries = (box is not None and reached_box and steps_done == len(seq)
                   and box.kind in BOX_TYPES and not on_top)
        if box is not None and reached_box:
            # the box sits still through the pick sequence (the plate may
            # overlap it); it goes onto the lift only once that is finished
            self.path_rule = (len(self.path) - 1 - steps_done, box.name, box.box)

        def on_arrival():
            self.engaged_box, self.engaged_sequence, self.engaged_steps = None, [], 0
            if box is None or not reached_box:
                return
            if carries:
                self.carry_origin = box
                self.carry_sequence = seq
                self.arena.attach(box.name, end)
            else:
                self.engaged_box, self.engaged_sequence, self.engaged_steps = box, seq, steps_done
        self.on_arrival = on_arrival

        if not reached_box:
            return " Couldn't reach the box itself - something's in the way."
        if seq and steps_done < len(seq):
            return (f" Stopped after step {steps_done + 1} of {len(seq)} in the pick "
                    "sequence - blocked, so the box won't be picked up.")
        if on_top:
            return (f" {box.name} has {', '.join(on_top)} on top, so it won't be "
                    "picked up - move that first.")
        if carries:
            return f" Once run, {box.name} is on the lift - click or drag on the map to place it."
        return ""

    def finish_place_path(self):
        """Tail of run() for a place Goal: append the set-down and drop
        (append_place()), and set self.on_arrival to leave the box where it
        was put, off the lift, with the plate engaged by whatever of the
        drop didn't run. Returns a note for the status bar."""
        box, seq = self.place_box, self.drop_pick_sequence()
        kind = self.arena.carried[0].kind
        placed, drop_done = self.append_place(seq)

        def on_arrival():
            if not placed:
                return                     # never got down there: still carrying it
            self.engaged_box = self.arena.release(box)
            self.engaged_sequence, self.engaged_steps = seq, len(seq) - drop_done
            self.carry_sequence, self.carry_origin, self.place_box = [], None, None
        self.on_arrival = on_arrival

        if not placed:
            return " Couldn't lower the box into place - something's in the way."
        if drop_done < len(seq):
            return (f" Stopped after step {drop_done + 1} of {len(seq)} in the drop "
                    "sequence - blocked.")
        if not seq:
            return (f" No pick sequence is set for '{kind}', so there is no drop "
                    "sequence to run - once run, the box is just let go.")
        return (f" Once run, the box is set down and the {len(seq)}-step drop "
                "sequence takes the plate off it.")

    def drop_pick_sequence(self):
        """The pick sequence whose reverse drops the carried box: the one
        that picked it up, or - if that was empty, say it was only filled
        in afterwards - the one set for its box type now."""
        return self.carry_sequence or self.sequence_for(self.arena.carried[0].kind)

    def append_steps(self, p, steps, ignore):
        """Append the relative moves `steps` ((axis, delta) pairs) to
        self.path, from `p`, each checked by the real geometry with
        straight_clear(ignore=ignore). Stops at the first blocked one and
        returns how many were appended."""
        d = self.arena.dims
        limits = (d.move_x, d.move_y, d.move_z)
        for i, (axis, delta) in enumerate(steps):
            nxt = list(p)
            nxt[axis] = snap(nxt[axis] + delta, 0.0, limits[axis])
            nxt = tuple(nxt)
            if not self.straight_clear(p, nxt, ignore=ignore):
                return i
            self.path.append(nxt)
            self.path_ignore.append(ignore)
            p = nxt
        return len(steps)

    def append_place(self, seq):
        """The place counterpart of append_final_contact(): from the Goal
        just above the spot, lower the carried box straight down until the
        next step would collide. That has to be the box's bottom meeting
        the surface under it (self.place_box); if anything else stops it
        first, or the lift bottoms out, it isn't placed. Then, with the box
        sitting still there, run the pick sequence `seq` in reverse to drop
        it (reverse_sequence()).

        Returns (placed, drop_done): placed is False if it couldn't be
        lowered onto the surface (the path is left above the spot, box
        still on the lift); drop_done is how many drop steps ran."""
        p = self.path[-1]
        while p[2] - RESOLUTION_CM >= -1e-9:
            nxt = (p[0], p[1], round((p[2] - RESOLUTION_CM) / RESOLUTION_CM) * RESOLUTION_CM)
            if self.in_collision(nxt):
                break
            p = nxt
        name, off = self.arena.carried[0].name, self.arena.carried[1]
        if abs(p[2] + off[2] - self.place_box[2]) > 1e-6:
            return False, 0        # stopped by something else, or never reached the surface
        if p != self.path[-1]:
            self.path.append(p)
            self.path_ignore.append(None)
        # from here the box sits still on the surface while the plate
        # leaves it; the drop is checked against it there - the plate may
        # touch it, nothing else may
        self.path_rule = (len(self.path) - 1, name, self.place_box)
        self.arena.box_override = (name, self.place_box)
        try:
            done = self.append_steps(p, reverse_sequence(seq), name)
        finally:
            self.arena.box_override = None
        return True, done

    def straight_clear(self, a, b, ignore=None):
        """True if the straight segment a-b is collision-free by the real,
        unclearanced geometry (not A*'s clearance-grown configuration
        space), sampled at the lattice resolution. Used for the short hops
        right at a box - approach, pick/drop sequence steps, and departure -
        where the plate is actually meant to get that close, closer than
        the general clearance setting allows. `ignore`: see in_collision()."""
        dist = math.dist(a, b)
        if dist < 1e-9:
            return True
        steps = max(1, math.ceil(dist / RESOLUTION_CM))
        return not any(self.in_collision(
            tuple(a[i] + (b[i] - a[i]) * k / steps for i in range(3)), ignore=ignore)
            for k in range(1, steps + 1))

    def escape_start(self, clearance, ignore=None, dirs=((0, 1, 0),)):
        """self.start, or - if it's blocked only by the clearance grown
        around an obstacle - the nearest point out along one of the
        directions `dirs`, tried in order at each distance (default +y,
        every box's pickup face; carrying a box, +z first to lift it off
        whatever it rests on) that clears that grown region, so plan_path() has a
        reachable point to search from instead of one sitting deep inside a
        blocked region it can never step out of. The fallback for when
        there's no tracked engagement to reverse, or disengage_start()
        itself is blocked; blind, so unlike disengage_start() it doesn't
        retrace the actual pick sequence - just heads +y.

        `ignore`: pass self.engaged_box.name (if any) so a start that's
        genuinely, deliberately overlapping that one box - per its pick
        sequence, since sequence steps are now allowed to do that - doesn't
        register as "something real is blocking me" on the very first step
        and give up immediately; every other obstacle still stops it.

        plan_path() already tolerates a start/goal sitting exactly on the
        boundary of the clearance-grown region (see astar_core._ExemptNodes);
        this covers being well inside it, which a single exempted lattice
        node can't fix since every neighbouring node is still blocked.
        """
        d = self.arena.dims
        obstacles = BoxObstacles(self.arena.configuration_boxes(clearance), RESOLUTION_CM)
        if cm_to_node(self.start) not in obstacles:
            return self.start
        limits = (d.move_x, d.move_y, d.move_z)
        alive = list(dirs)   # directions not yet stopped by a limit or a collision
        k = 0
        while alive:
            k += 1
            for v in list(alive):
                pt = tuple(self.start[i] + v[i] * k * RESOLUTION_CM for i in range(3))
                if (any(not -1e-9 <= pt[i] <= limits[i] + 1e-9 for i in range(3))
                        or self.in_collision(pt, ignore=ignore)):
                    alive.remove(v)   # off the travel, or something else is in the way
                elif cm_to_node(pt) not in obstacles:
                    return pt
        return self.start   # couldn't find a way clear; plan_path will report it

    def infer_engagement(self):
        """If self.engaged_box isn't already tracking an engagement - Start
        was just synced from real hardware that's resting against a box
        from an earlier session, say, rather than set here by this
        session's own append_final_contact() - work it out from the
        physical collision itself: if Start's only collision is with
        exactly one box, that's assumed to be an intentional engagement
        rather than a real obstruction, matching what running that box
        type's pick sequence in full would leave. That both stops it being
        reported as "Start is in collision" and gives disengage_start()
        something to reverse before planning a move away from it."""
        if self.engaged_box is not None or self.arena.carried is not None:
            return
        saved = list(self.arena.pos)
        self.arena.move_to(*self.start)
        hits = self.arena.collisions()
        self.arena.pos[:] = saved
        names = {s for _, s in hits}
        if len(names) != 1:
            return   # nothing touching, or touching more than one thing
        box = next((o for o in self.arena.obstacles if o.name in names), None)
        if box is None:
            return   # touching a wall, not a box - a real obstruction
        self.engaged_box = box
        self.engaged_sequence = self.sequence_for(box.kind)
        self.engaged_steps = len(self.engaged_sequence)

    def disengage_start(self, clearance):
        """If self.engaged_box says the plate is currently touching/
        overlapping a box per its pick sequence, the drop for that box -
        always just the pick sequence reversed (reverse_sequence()), run on
        however much of it actually completed (engaged_sequence[:
        engaged_steps]) - run from self.start, each step allowed to touch
        that one box (in_collision's ignore) same as picking it up did,
        landing back at its exact contact point, then the usual standoff
        move (not allowed to touch anything) to clear A*'s search space.

        Returns (plan_start, prepend_points, prepend_ignore) like
        plan_start_and_prepend(), or None if there's no engagement tracked
        or a step is blocked by something other than that box - in which
        case plan_start_and_prepend() falls back to escape_start()."""
        box = self.engaged_box
        if box is None:
            return None
        name = box.name
        d = self.arena.dims
        limits = (d.move_x, d.move_y, d.move_z)
        pts, ignore = [self.start], [name]   # self.start is exactly where that engagement left the plate
        p = self.start
        for axis, delta in reverse_sequence(self.engaged_sequence[:self.engaged_steps]):
            nxt = list(p)
            nxt[axis] = snap(nxt[axis] + delta, 0.0, limits[axis])
            nxt = tuple(nxt)
            if not self.straight_clear(p, nxt, ignore=name):
                return None
            pts.append(nxt)
            ignore.append(name)
            p = nxt
        standoff = self.box_offset_point(box, clearance + RESOLUTION_CM)
        if not self.straight_clear(p, standoff):
            return None
        return standoff, pts, ignore

    def plan_start_and_prepend(self, clearance):
        """The point to actually search from, plus any points (and their
        collision-check exemptions, for run()'s validity check) to prepend
        to the resulting path to bridge from self.start to it. Tries
        disengage_start() first - exact, since it retraces the engagement
        that put self.start where it is - then falls back to
        escape_start()'s blind step along +y. Carrying a box, it just lifts
        it clear (+z) - reversing the pick sequence would drop it."""
        if self.arena.carried is not None:
            # up first; if the carriage is too near the face of the box it
            # was lifted off, which it fills the full height beside, going
            # up alone never clears it - so then up and back, then back
            plan_start = self.escape_start(clearance, dirs=((0, 0, 1), (0, 1, 1), (0, 1, 0)))
            if plan_start == self.start:
                return plan_start, [], []
            return plan_start, [self.start], [None]
        out = self.disengage_start(clearance)
        if out is not None:
            return out
        name = self.engaged_box.name if self.engaged_box else None
        plan_start = self.escape_start(clearance, ignore=name)
        if plan_start == self.start:
            return plan_start, [], []
        return plan_start, [self.start], [name]

    def append_final_contact(self):
        """box_goal_point() backs the Goal off the box by a standoff so A*
        can actually reach it (see its docstring); this closes that last
        bit with a short straight move so the plate ends up at
        box_exact_point() - touching the box - instead of stopping short at
        the standoff, then runs self.final_sequence (the box type's pick
        sequence) from there, one step at a time.

        Each step, including the initial approach, is checked with the
        real, unclearanced geometry via straight_clear(), allowed to touch
        or overlap self.target_box (in_collision's ignore) since that's the
        point of engaging it - every other obstacle still blocks it. The
        first step that would collide some other way stops the sequence
        there, keeping everything appended up to that point rather than
        failing the whole path.

        Returns (reached_box, steps_done): reached_box is False if even the
        approach to the box itself was blocked (self.path is left at the
        standoff); steps_done is how many of self.final_sequence's steps
        were completed before one was blocked (or the full count, if none
        was). run() uses these to report a sequence that got cut short,
        since otherwise it happens silently, and to update engagement
        tracking for the next disengage_start()."""
        if not self.path or self.final_point is None:
            return True, 0
        ignore = self.target_box.name if self.target_box is not None else None
        last, target = self.path[-1], self.final_point
        if not all(abs(last[i] - target[i]) < 1e-9 for i in range(3)):
            if not self.straight_clear(last, target, ignore=ignore):
                return False, 0   # can't even reach the box; left at the standoff
            self.path.append(target)
            self.path_ignore.append(ignore)
        return True, self.append_steps(target, self.final_sequence, ignore)

    def reset_goal(self):
        self.goal = None
        self.target_box = None
        self.final_point = None
        self.final_sequence = []
        self.place_box = None
        self.invalidate()

    def clear(self):
        self.reset_goal()
        self.engaged_box, self.engaged_sequence, self.engaged_steps = None, [], 0
        if self.arena.carried is not None and not self.link.connected:
            # simulated pick: put the box back where it came from - or, if
            # it was loaded straight onto the lift, take it away
            if self.carry_origin is not None:
                self.arena.release(self.carry_origin.box)
            else:
                self.arena.carried = None
            self.carry_sequence, self.carry_origin = [], None
        self.arena.home()
        if not self.sync_start():        # not connected: Start is the home pose
            self.start = tuple(self.arena.pos)
        self.status.set("Cleared.")
        self.draw()

    # ---- 3D view controls -----------------------------------------------

    def on_view_press(self, event):
        self._view_drag = (event.x, event.y)
        self._view_press = (event.x, event.y)

    def on_view_drag(self, event):
        if self._view_drag is None:
            return
        dx, dy = event.x - self._view_drag[0], event.y - self._view_drag[1]
        self._view_drag = (event.x, event.y)
        self.yaw += dx * 0.01
        self.pitch = max(0.05, min(math.pi / 2, self.pitch + dy * 0.01))
        self._view_press = None      # this was a drag, not a click
        self.draw3d()

    def on_view_release(self, event):
        press, self._view_drag, self._view_press = self._view_press, None, None
        if press is not None:        # the mouse barely moved: treat as a click
            self.pick_in_view(event.x, event.y)

    def zoom_by(self, factor):
        self.zoom = max(0.5, min(12.0, self.zoom * factor))
        self.draw3d()

    def project(self, x, y, z):
        """cm -> (screen x, screen y, depth). Larger depth is farther away."""
        d = self.arena.dims
        ix0, iy0, ix1, iy1 = self.arena.inner_box()
        X = x - (ix0 + ix1) / 2
        Y = y - (iy0 + iy1) / 2
        Z = z - d.wall_height / 2
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        x1 = X * cy - Y * sy
        y1 = X * sy + Y * cy
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        y2 = y1 * cp - Z * sp
        z2 = y1 * sp + Z * cp
        return (VIEW_W / 2 + x1 * self.zoom, VIEW_H / 2 - z2 * self.zoom, y2)

    def unproject(self, sx, sy, world_z):
        """Inverse of project(), for one chosen world z: the (x, y) where
        the camera ray through screen point (sx, sy) crosses the horizontal
        plane at that height. project() is orthographic (no perspective
        divide), so a screen point maps to a line in world space, not a
        point; fixing world_z picks the one point on that line we want."""
        d = self.arena.dims
        ix0, iy0, ix1, iy1 = self.arena.inner_box()
        cx, cy = (ix0 + ix1) / 2, (iy0 + iy1) / 2
        Z = world_z - d.wall_height / 2
        cyaw, syaw = math.cos(self.yaw), math.sin(self.yaw)
        cpitch, spitch = math.cos(self.pitch), math.sin(self.pitch)
        x1 = (sx - VIEW_W / 2) / self.zoom
        z2 = (VIEW_H / 2 - sy) / self.zoom
        y1 = (z2 - Z * cpitch) / spitch
        X = x1 * cyaw + y1 * syaw
        Y = -x1 * syaw + y1 * cyaw
        return (X + cx, Y + cy)

    def pick_in_view(self, sx, sy):
        """A click in the 3D view: snap to a box's lift point if the click
        landed near its marker, otherwise aim at the floor under the
        cursor (at the current Lift z) - or, carrying a box, place it
        there - the 3D equivalent of clicking the 2D map."""
        for o in self.arena.obstacles:
            mx, my, _ = self.project(*self.arena.box_lift_point(o))
            if (mx - sx) ** 2 + (my - sy) ** 2 <= 64:   # within 8 px
                self.set_goal(self.box_goal_point(o), o)
                return
        d = self.arena.dims
        x, y = self.unproject(sx, sy, 0.0)
        if self.arena.carried is not None:   # place the carried box, centred there
            off = self.arena.carried[1]
            self.set_place_goal(snap(x - (off[3] - off[0]) / 2, -1e6, 1e6),
                                snap(y - (off[4] - off[1]) / 2, -1e6, 1e6))
            return
        point = (snap(x, 0.0, d.move_x), snap(y, 0.0, d.move_y),
                 snap(self.z.get(), 0.0, d.move_z))
        self.set_goal(point)

    # ---- drawing --------------------------------------------------------

    def shown_hit_names(self):
        """Names of everything to draw red: what the mechanism overlaps at
        the current pose, except the box a pick/drop sequence is lifting or
        setting down (box_override), which the plate is meant to engage."""
        a = self.arena
        ok = a.box_override[0] if a.box_override else None
        return {n for pair in a.collisions() if ok not in pair for n in pair}

    def draw(self):
        self.update_readout()
        self.draw_map()
        self.draw3d()

    def update_readout(self):
        """Readout of the pose the views show: the tool point, and heights
        above the arena floor of the lift plate (the orange one) and of the
        bottom of a box on the lift."""
        a = self.arena
        x, y, z = a.pos
        plate = a.plate_box(x, y, z)
        text = (f"Tool x {x:7.2f}  y {y:7.2f}  lift z {z:6.2f} cm    "
                f"Plate above floor: underside {plate[2]:6.2f}  top {plate[5]:6.2f} cm")
        carried = a.carried_solid(x, y, z)
        if carried and not (a.box_override and a.box_override[0] == carried.name):
            text += f"    {carried.name} bottom above floor {carried.box[2]:6.2f} cm"
        self.readout.set(text)

    def draw_map(self):
        c = self.canvas
        c.delete("all")
        a = self.arena
        d = a.dims
        ix0, iy0, ix1, iy1 = a.inner_box()
        t = d.wall_thickness

        # walls, floor, moveable area
        c.create_rectangle(self.sx(ix0 - t), self.sy(iy1 + t),
                           self.sx(ix1 + t), self.sy(iy0 - t),
                           fill="#6b6b6b", outline="")
        c.create_rectangle(self.sx(ix0), self.sy(iy1), self.sx(ix1), self.sy(iy0),
                           fill="#222222", outline="")
        for gx in range(0, int(ix1) + 1, 10):
            c.create_line(self.sx(gx), self.sy(iy1), self.sx(gx), self.sy(iy0), fill="#2b2b2b")
        for gy in range(0, int(iy1) + 1, 10):
            c.create_line(self.sx(ix0), self.sy(gy), self.sx(ix1), self.sy(gy), fill="#2b2b2b")
        c.create_rectangle(self.sx(0), self.sy(d.move_y), self.sx(d.move_x), self.sy(0),
                           outline=LIMIT_COLOR, dash=(4, 3))
        # the overhead wall: hatched, since only a plate or box that's
        # high enough hits it
        b = a.overhead_wall().box
        c.create_rectangle(self.sx(b[0]), self.sy(b[4]), self.sx(b[3]), self.sy(b[1]),
                           fill="#6b6b6b", stipple="gray50", outline="#8a8a8a")
        c.create_text(self.sx(ix1) - 4, self.sy(b[1]) + 2, anchor="ne", fill="#9a9a9a",
                      text=f"overhead wall, z {b[2]:g} and up", font=("TkDefaultFont", 7))

        # boxes, lowest first so stacked boxes draw on top of what they sit on
        for o in sorted(a.boxes(), key=lambda o: o.box[2]):
            b = o.box
            c.create_rectangle(self.sx(b[0]), self.sy(b[4]), self.sx(b[3]), self.sy(b[1]),
                               fill=KIND_COLOR.get(o.kind, "#8a6a3a"), outline="#d8d8d8")
            c.create_text((self.sx(b[0]) + self.sx(b[3])) / 2,
                          (self.sy(b[1]) + self.sy(b[4])) / 2,
                          text=f"{b[5]:g}", fill="black", font=("TkDefaultFont", 8))

        # path
        if len(self.path) >= 2:
            pts = [v for p in self.path for v in (self.sx(p[0]), self.sy(p[1]))]
            c.create_line(pts, fill=PATH_COLOR, width=2)

        # start / goal
        for cell, color in ((self.start, "#3c96ff"), (self.goal, "#f05032")):
            if cell:
                r = 5
                px, py = self.sx(cell[0]), self.sy(cell[1])
                c.create_oval(px - r, py - r, px + r, py + r, fill=color, outline="white")

        # the mechanism at its current pose
        hit_names = self.shown_hit_names()
        for sol in a.moving_solids():
            b = sol.box
            c.create_rectangle(self.sx(b[0]), self.sy(b[4]), self.sx(b[3]), self.sy(b[1]),
                               fill=HIT_COLOR if sol.name in hit_names
                               else KIND_COLOR.get(sol.kind, "#8a6a3a"),
                               outline="white", stipple="gray50" if sol.kind == "carriage" else "")

        # where the carried box is being placed: red if it can't go there
        if self.place_box is not None:
            b = self.place_box
            c.create_rectangle(self.sx(b[0]), self.sy(b[4]), self.sx(b[3]), self.sy(b[1]),
                               outline=PLACE_COLOR if self.goal else HIT_COLOR,
                               width=2, dash=(5, 3))
            c.create_text((self.sx(b[0]) + self.sx(b[3])) / 2,
                          (self.sy(b[1]) + self.sy(b[4])) / 2,
                          text=f"{b[5]:g}", fill=PLACE_COLOR, font=("TkDefaultFont", 8))

        # lift points
        for o in a.boxes():
            lx, ly, _ = a.box_lift_point(o)
            px, py, r = self.sx(lx), self.sy(ly), 3
            c.create_oval(px - r, py - r, px + r, py + r, fill="#ffd23c", outline="black")
        lx, ly, _ = a.plate_lift_point(*a.pos)
        px, py, r = self.sx(lx), self.sy(ly), 3
        c.create_oval(px - r, py - r, px + r, py + r, fill="#ff5fb0", outline="black")

    def faces(self, box, color):
        """The six faces of a box as (depth, screen points, color)."""
        p = self.project
        x0, y0, z0, x1, y1, z1 = box
        q = {(i, j, k): p(x1 if i else x0, y1 if j else y0, z1 if k else z0)
             for i in (0, 1) for j in (0, 1) for k in (0, 1)}
        out = []
        for axis in range(3):
            for side in (0, 1):
                quad = [q[k] for k in q if k[axis] == side]
                cx = sum(v[0] for v in quad) / 4
                cy = sum(v[1] for v in quad) / 4
                quad.sort(key=lambda v: math.atan2(v[1] - cy, v[0] - cx))
                out.append((sum(v[2] for v in quad) / 4,
                            [n for v in quad for n in v[:2]], color))
        return out

    def draw3d(self):
        v = self.view
        v.delete("all")
        a = self.arena
        d = a.dims
        p = self.project
        m = d.lift_floor_margin

        faces = []
        for w in a.walls():
            faces += self.faces(w.box, "#5a5a5a" if w.name == "floor" else "#7a7a7a")
        for o in a.boxes():
            faces += self.faces(o.box, KIND_COLOR.get(o.kind, "#8a6a3a"))
        hit_names = self.shown_hit_names()
        for sol in a.moving_solids():
            faces += self.faces(sol.box, HIT_COLOR if sol.name in hit_names
                                else KIND_COLOR.get(sol.kind, "#8a6a3a"))
        if self.place_box is not None:
            faces += self.faces(self.place_box, PLACE_COLOR if self.goal else HIT_COLOR)

        if self.explored:
            step = max(1, len(self.explored) // MAX_EXPLORED_DOTS)
            for i, (x, y, z) in enumerate(self.explored):
                if i % step == 0:
                    sx, sy, _ = p(x, y, z + m)
                    v.create_oval(sx - 1, sy - 1, sx + 1, sy + 1,
                                  fill="#6a6a6a", outline="")

        # wireframe only (no fill): a solid face would hide whatever is
        # behind it, and painter's-algorithm depth sorting of flat quads
        # can't be made reliable enough to guarantee every object's faces
        # stay visible from every angle, so nothing is ever opaque here.
        for _, pts, color in sorted(faces, key=lambda f: -f[0]):
            v.create_polygon(pts, fill="", outline=color, width=1)

        # the moveable area and lift limits: the box the tool point can
        # reach, drawn at the plate underside's height like the path (lift
        # z 0 to move_z, i.e. lift_floor_margin to that plus move_z)
        corners = {(i, j, k): p(d.move_x * i, d.move_y * j, m + d.move_z * k)[:2]
                   for i in (0, 1) for j in (0, 1) for k in (0, 1)}
        for a_, b_ in ((a_, b_) for a_ in corners for b_ in corners
                       if a_ < b_ and sum(x != y for x, y in zip(a_, b_)) == 1):
            v.create_line(*corners[a_], *corners[b_], fill=LIMIT_COLOR, dash=(4, 3))

        if len(self.path) >= 2:
            pts = [n for q in self.path for n in p(q[0], q[1], q[2] + m)[:2]]
            v.create_line(pts, fill=PATH_COLOR, width=3, joinstyle="round")

        for cell, color, label in ((self.start, "#3c96ff", "S"),
                                   (self.goal, "#f05032", "G")):
            if cell:
                sx, sy, _ = p(cell[0], cell[1], cell[2] + m)
                r = 7
                v.create_oval(sx - r, sy - r, sx + r, sy + r, fill=color, outline="white")
                v.create_text(sx, sy, text=label, fill="white",
                              font=("TkDefaultFont", 8, "bold"))

        # lift points
        for o in a.boxes():
            sx, sy, _ = p(*a.box_lift_point(o))
            r = 3
            v.create_oval(sx - r, sy - r, sx + r, sy + r, fill="#ffd23c", outline="black")
        sx, sy, _ = p(*a.plate_lift_point(*a.pos))
        r = 3
        v.create_oval(sx - r, sy - r, sx + r, sy + r, fill="#ff5fb0", outline="black")

        v.create_text(8, VIEW_H - 8, anchor="sw", fill="#777777",
                      text="drag: rotate   wheel: zoom   click: set Goal"
                           + ("   (drag the carried box on the map to place it)"
                              if a.carried else ""))


if __name__ == "__main__":
    root = tk.Tk()
    AStarGui(root)
    root.mainloop()

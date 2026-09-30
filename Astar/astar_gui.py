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
the carriage and lift along the planned path.

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

from arena_env import (BOX_TYPES, HIT_COLOR, KIND_COLOR, Arena, make_boxes)
from astar_core import RESOLUTION_CM, BoxObstacles, cm_to_node, plan_path, smooth_path
from esp_link import DEFAULT_SPEED_CM_S, EspLink, LinkError

PX = 3                        # map pixels per cm
VIEW_W = 520
VIEW_H = 480
MAX_EXPLORED_DOTS = 4000
PATH_COLOR = "#46b478"


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

        self.path = []
        self.path_ignore = []
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

        # per box type, the sequence of tool-point moves to run once the
        # plate reaches that box's exact lift point, to pick it up (see
        # append_final_contact and sequence_for()). Steps may touch/overlap
        # that box - see in_collision()'s ignore - since that's the point of
        # engaging it; every other obstacle still blocks them. There's no
        # separate drop sequence to configure: leaving an engaged box always
        # runs this same sequence in reverse to disengage (disengage_start()).
        self.pick_seq = {kind: tk.StringVar(value="") for kind in BOX_TYPES}

        seqframe = tk.LabelFrame(
            root, text='Pick sequence per box type, run once the plate reaches '
                       'the box - e.g. "y1, z-0.5" (axis + signed cm, in order). '
                       "Leaving an engaged box automatically reverses it to "
                       "disengage.")
        seqframe.pack(fill="x", padx=8, pady=(0, 8))
        tk.Label(seqframe, text="Box type").grid(row=0, column=0, padx=(4, 4))
        tk.Label(seqframe, text="Pick sequence").grid(row=0, column=1, padx=4)
        for r, kind in enumerate(BOX_TYPES, start=1):
            tk.Label(seqframe, text=kind).grid(row=r, column=0, sticky="w", padx=(4, 4))
            tk.Entry(seqframe, textvariable=self.pick_seq[kind], width=20
                     ).grid(row=r, column=1, padx=4, pady=1, sticky="w")

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

        mbar = tk.Frame(root)
        mbar.pack(fill="x", padx=8, pady=(6, 0))
        tk.Label(mbar, text="Machine host:").pack(side="left")
        self.port = tk.StringVar(value="stepper.local")
        tk.Entry(mbar, textvariable=self.port, width=18).pack(side="left", padx=4)
        self.connect_btn = tk.Button(mbar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=2)
        tk.Label(mbar, text="Speed (cm/s):").pack(side="left", padx=(12, 0))
        self.speed = tk.DoubleVar(value=DEFAULT_SPEED_CM_S)
        tk.Spinbox(mbar, from_=0.5, to=20, increment=0.5, width=5,
                   textvariable=self.speed).pack(side="left", padx=4)
        self.send_btn = tk.Button(mbar, text="Send path to machine",
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
        self.canvas.bind("<B1-Motion>", self.on_press)
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
                  "then Run A*.")
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
                    self.arena.move_to(*val)
                    moved = True
                elif kind == "done":
                    self.running = False
                    self.send_btn.config(state="normal")
                    self.status.set(val)
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
        if not self.link.connected:
            self.status.set("Connect to the machine first.")
            return
        path, speed = list(self.path), self.speed.get()
        self.running = True
        self.send_btn.config(state="disabled")
        self.status.set("Following the path... (STOP aborts)")

        def work():
            try:
                self.link.follow(path, speed,
                                 progress=lambda p: self.events.put(("pose", p)))
                self.events.put(("done", "Arrived at the goal."))
            except LinkError as e:
                self.events.put(("done", f"Path not completed: {e}"))
            except Exception:         # a bug must not leave the GUI "following" forever
                self.events.put(("log", traceback.format_exc()))
                self.events.put(("done", "Path aborted by an internal error - see the "
                                         "controller messages below."))
                self.link.stop()
        threading.Thread(target=work, daemon=True).start()

    def stop_machine(self):
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

    def invalidate(self):
        self.path = []
        self.path_ignore = []  # parallel to self.path: a solid name to excuse
                                # from that point's collision check, or None
        self.explored = set()
        self.scrub.config(state="disabled", to=0)
        self.scrub.set(0)

    # ---- editing --------------------------------------------------------

    def on_motion(self, event):
        x, y = self.cm_at(event)
        self.cursor.set(f"x {x:g}  y {y:g} cm")

    def box_offset_point(self, box, offset):
        """The tool point that puts the plate's lift point `offset` cm out
        from `box`'s lift point, along the box's back face normal (+y):
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
                return o, self.box_goal_point(b)
        return None

    def set_goal(self, point, box=None):
        """Common tail for setting the Goal from either view: update state,
        check for collision there, and report status."""
        self.goal = point
        self.target_box = box
        if box is not None:
            self.final_point = self.box_exact_point(box.box)
            self.final_sequence = self.sequence_for(box.kind)
        else:
            self.final_point = point
            self.final_sequence = []
        self.invalidate()
        self.arena.move_to(*point)  # show the mechanism where it was put
        hits = self.arena.collisions()
        if hits:
            self.status.set("In collision here: "
                            + "; ".join(f"{m} / {s}" for m, s in hits))
        elif box is not None:
            self.status.set(f"Goal set to lift {box.name} - carriage "
                            f"({point[0]:g}, {point[1]:g}), lift z {point[2]:g}.")
        else:
            self.status.set(f"Goal set to ({point[0]:g}, {point[1]:g}, {point[2]:g}).")
        self.draw()

    def on_press(self, event):
        x, y = self.cm_at(event)
        self.on_motion(event)
        box_hit = self.goal_for_box(x, y)
        if box_hit is not None:
            box, point = box_hit
        else:
            box, point = None, (x, y, snap(self.z.get(), 0.0, self.arena.dims.move_z))
        self.set_goal(point, box)

    def on_scrub(self, value):
        if self.path:
            self.arena.move_to(*self.path[int(float(value))])
            self.draw()

    def run(self):
        self.sync_start()
        self.infer_engagement()
        if self.goal is None:
            self.status.set("Set a Goal first.")
            return
        for name, p in (("Start", self.start), ("Goal", self.goal)):
            if self.in_collision(p, ignore=self.engaged_box.name if self.engaged_box else None):
                self.status.set(f"{name} is in collision with a box, so no "
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
        self.path = plan_path(
            self.arena, plan_start, self.goal, diagonals=self.diagonals.get(),
            explored=self.explored if self.show_explored.get() else None,
            clearance=clearance)
        # the lattice staircase makes the motors wiggle: pull it into straight runs
        self.path = smooth_path(self.arena, self.path, clearance=clearance)
        if self.path:
            self.path_ignore = [None] * len(self.path)
            if prepend_pts:
                self.path = prepend_pts + self.path
                self.path_ignore = prepend_ignore + self.path_ignore
        sequence_note = ""
        if self.path:
            reached_box, steps_done = self.append_final_contact()
            if self.target_box is not None and reached_box:
                self.engaged_box = self.target_box
                self.engaged_sequence = self.final_sequence
                self.engaged_steps = steps_done
            else:
                self.engaged_box, self.engaged_sequence, self.engaged_steps = None, [], 0
            if not reached_box:
                sequence_note = " Couldn't reach the box itself - something's in the way."
            elif self.final_sequence and steps_done < len(self.final_sequence):
                sequence_note = (f" Stopped after step {steps_done + 1} of "
                                 f"{len(self.final_sequence)} in the pick "
                                 "sequence - blocked.")
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
            self.arena.move_to(*self.path[0])
        else:
            self.status.set(f"No path found ({dt:.1f} s).")
        self.draw()

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

    def escape_start(self, clearance, ignore=None):
        """self.start, or - if it's blocked only by the clearance grown
        around an obstacle - the nearest point out along +y (every box's
        pickup face) that clears that grown region, so plan_path() has a
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
        p = list(self.start)
        while p[1] < d.move_y:
            p[1] = min(p[1] + RESOLUTION_CM, d.move_y)
            pt = tuple(p)
            if self.in_collision(pt, ignore=ignore):
                break   # something else is in the way; give up escaping
            if cm_to_node(pt) not in obstacles:
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
        if self.engaged_box is not None:
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
        standoff = self.box_offset_point(box.box, clearance + RESOLUTION_CM)
        if not self.straight_clear(p, standoff):
            return None
        return standoff, pts, ignore

    def plan_start_and_prepend(self, clearance):
        """The point to actually search from, plus any points (and their
        collision-check exemptions, for run()'s validity check) to prepend
        to the resulting path to bridge from self.start to it. Tries
        disengage_start() first - exact, since it retraces the engagement
        that put self.start where it is - then falls back to
        escape_start()'s blind step along +y."""
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
        d = self.arena.dims
        limits = (d.move_x, d.move_y, d.move_z)
        p = target
        for i, (axis, delta) in enumerate(self.final_sequence):
            nxt = list(p)
            nxt[axis] = snap(nxt[axis] + delta, 0.0, limits[axis])
            nxt = tuple(nxt)
            if not self.straight_clear(p, nxt, ignore=ignore):
                return True, i   # this step is blocked; stop the sequence here
            self.path.append(nxt)
            self.path_ignore.append(ignore)
            p = nxt
        return True, len(self.final_sequence)

    def clear(self):
        self.goal = None
        self.target_box = None
        self.final_point = None
        self.final_sequence = []
        self.engaged_box, self.engaged_sequence, self.engaged_steps = None, [], 0
        self.invalidate()
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
        cursor (at the current Lift z), the 3D equivalent of clicking the
        2D map."""
        for o in self.arena.obstacles:
            mx, my, _ = self.project(*self.arena.box_lift_point(o.box))
            if (mx - sx) ** 2 + (my - sy) ** 2 <= 64:   # within 8 px
                self.set_goal(self.box_goal_point(o.box), o)
                return
        d = self.arena.dims
        x, y = self.unproject(sx, sy, 0.0)
        point = (snap(x, 0.0, d.move_x), snap(y, 0.0, d.move_y),
                 snap(self.z.get(), 0.0, d.move_z))
        self.set_goal(point)

    # ---- drawing --------------------------------------------------------

    def draw(self):
        self.draw_map()
        self.draw3d()

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
                           outline="#4a7db5", dash=(4, 3))

        # boxes, lowest first so stacked boxes draw on top of what they sit on
        for o in sorted(a.obstacles, key=lambda o: o.box[2]):
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
        hits = a.collisions()
        hit_names = {n for pair in hits for n in pair}
        for sol in a.moving_solids():
            b = sol.box
            c.create_rectangle(self.sx(b[0]), self.sy(b[4]), self.sx(b[3]), self.sy(b[1]),
                               fill=HIT_COLOR if sol.name in hit_names else KIND_COLOR[sol.kind],
                               outline="white", stipple="gray50" if sol.kind == "carriage" else "")

        # lift points
        for o in a.obstacles:
            lx, ly, _ = a.box_lift_point(o.box)
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
        for o in a.obstacles:
            faces += self.faces(o.box, KIND_COLOR.get(o.kind, "#8a6a3a"))
        hit_names = {n for pair in a.collisions() for n in pair}
        for sol in a.moving_solids():
            faces += self.faces(sol.box,
                                HIT_COLOR if sol.name in hit_names else KIND_COLOR[sol.kind])

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
        for o in a.obstacles:
            sx, sy, _ = p(*a.box_lift_point(o.box))
            r = 3
            v.create_oval(sx - r, sy - r, sx + r, sy + r, fill="#ffd23c", outline="black")
        sx, sy, _ = p(*a.plate_lift_point(*a.pos))
        r = 3
        v.create_oval(sx - r, sy - r, sx + r, sy + r, fill="#ff5fb0", outline="black")

        v.create_text(8, VIEW_H - 8, anchor="sw", fill="#777777",
                      text="drag: rotate   wheel: zoom   click: set Goal")


if __name__ == "__main__":
    root = tk.Tk()
    AStarGui(root)
    root.mainloop()

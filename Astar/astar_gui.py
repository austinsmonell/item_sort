"""GUI for the 3D A* pathfinder, built with tkinter (Python standard library
- nothing to install). It plans through the environment defined in
arena_env.py (walls, boxes, carriage and lift) using plan_path() from
astar_core.py - the GUI runs the real algorithm, it does not reimplement it.

Left: a top-down map (x right, y up, x = 0 y = 0 at the bottom left of the
moveable area). Pick Start or Goal and click to place it; it goes at the
lift reading set by the Lift z slider. The carriage and lift plate are drawn
at their current pose and turn red if they overlap anything. Boxes show
their top height in cm.

Run A* plans the tool point (carriage x, carriage y, lift z) around every box,
allowing for the size of the carriage and plate. The Path slider then steps
the carriage and lift along the planned path.

Right: a 3D view of the whole arena. Drag to rotate, mouse wheel to zoom.

Change the arena (dimensions, boxes) in arena_env.py.

Run with:  python astar_gui.py
"""

import math
import time
import tkinter as tk

from arena_env import (HIT_COLOR, KIND_COLOR, Arena, make_boxes)
from astar_core import RESOLUTION_CM, plan_path

PX = 3                        # map pixels per cm
VIEW_W = 520
VIEW_H = 480
MAX_EXPLORED_DOTS = 4000
PATH_COLOR = "#46b478"


def snap(v, lo, hi):
    """Round a cm value to the resolution and keep it within lo..hi."""
    return min(max(round(v / RESOLUTION_CM) * RESOLUTION_CM, lo), hi)


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

        self.mode = tk.StringVar(value="start")
        self.z = tk.DoubleVar(value=d.move_z)
        self.diagonals = tk.BooleanVar(value=True)
        self.show_explored = tk.BooleanVar(value=False)
        self.cursor = tk.StringVar(value="")

        self.start = None        # (x, y, z) tool point, cm
        self.goal = None
        self.path = []
        self.explored = set()

        self.yaw = math.radians(35)
        self.pitch = math.radians(55)
        self.zoom = 2.2          # 3D view pixels per cm
        self._view_drag = None

        toolbar = tk.Frame(root)
        toolbar.pack(fill="x", padx=8, pady=8)
        for text, value in [("Start", "start"), ("Goal", "goal")]:
            tk.Radiobutton(toolbar, text=text, value=value, variable=self.mode,
                           indicatoron=False, width=8).pack(side="left", padx=2)
        tk.Checkbutton(toolbar, text="Diagonals", variable=self.diagonals
                       ).pack(side="left", padx=(12, 2))
        tk.Checkbutton(toolbar, text="Show explored (3D)",
                       variable=self.show_explored).pack(side="left", padx=2)
        tk.Button(toolbar, text="Run A*", command=self.run).pack(side="left", padx=(12, 2))
        tk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=2)

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
        self.view.bind("<MouseWheel>",
                       lambda e: self.zoom_by(1.1 if e.delta > 0 else 1 / 1.1))
        self.view.bind("<Button-4>", lambda e: self.zoom_by(1.1))
        self.view.bind("<Button-5>", lambda e: self.zoom_by(1 / 1.1))

        self.status = tk.StringVar(
            value=f"Moveable area {d.move_x:g} x {d.move_y:g} cm, lift z 0 to "
                  f"{d.move_z:g}, {RESOLUTION_CM:g} cm resolution. Click the "
                  "map to set Start and Goal, then Run A*.")
        tk.Label(root, textvariable=self.status, anchor="w", justify="left",
                 wraplength=1060).pack(fill="x", padx=8, pady=8)

        self.arena.home()
        self.draw()

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

    def in_collision(self, point):
        saved = list(self.arena.pos)
        self.arena.move_to(*point)
        hit = bool(self.arena.collisions())
        self.arena.pos[:] = saved
        return hit

    def invalidate(self):
        self.path = []
        self.explored = set()
        self.scrub.config(state="disabled", to=0)
        self.scrub.set(0)

    # ---- editing --------------------------------------------------------

    def on_motion(self, event):
        x, y = self.cm_at(event)
        self.cursor.set(f"x {x:g}  y {y:g} cm")

    def on_press(self, event):
        x, y = self.cm_at(event)
        self.on_motion(event)
        point = (x, y, snap(self.z.get(), 0.0, self.arena.dims.move_z))
        if self.mode.get() == "start":
            self.start = point
        else:
            self.goal = point
        self.invalidate()
        self.arena.move_to(*point)  # show the mechanism where it was put
        hits = self.arena.collisions()
        if hits:
            self.status.set("In collision here: "
                            + "; ".join(f"{m} / {s}" for m, s in hits))
        else:
            self.status.set(f"{self.mode.get().capitalize()} set to "
                            f"({point[0]:g}, {point[1]:g}, {point[2]:g}).")
        self.draw()

    def on_scrub(self, value):
        if self.path:
            self.arena.move_to(*self.path[int(float(value))])
            self.draw()

    def run(self):
        if self.start is None or self.goal is None:
            self.status.set("Set both a Start and a Goal first.")
            return
        for name, p in (("Start", self.start), ("Goal", self.goal)):
            if self.in_collision(p):
                self.status.set(f"{name} is in collision with a box, so no "
                                "path can start or end there. Move it.")
                return
        self.status.set("Planning...")
        self.root.update_idletasks()
        self.explored = set()
        t = time.time()
        self.path = plan_path(
            self.arena, self.start, self.goal, diagonals=self.diagonals.get(),
            explored=self.explored if self.show_explored.get() else None)
        dt = time.time() - t
        if self.path:
            length = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
            bad = sum(1 for p in self.path if self.in_collision(p))
            check = ("collision-free" if not bad
                     else f"WARNING: {bad} points collide")
            self.status.set(f"Path found: {len(self.path)} points, {length:.1f} cm, "
                            f"{check} ({dt:.1f} s). Drag the Path slider to step "
                            "through it.")
            self.scrub.config(state="normal", to=len(self.path) - 1)
            self.scrub.set(0)
            self.arena.move_to(*self.path[0])
        else:
            self.status.set(f"No path found ({dt:.1f} s).")
        self.draw()

    def clear(self):
        self.start = self.goal = None
        self.invalidate()
        self.arena.home()
        self.status.set("Cleared.")
        self.draw()

    # ---- 3D view controls -----------------------------------------------

    def on_view_press(self, event):
        self._view_drag = (event.x, event.y)

    def on_view_drag(self, event):
        if self._view_drag is None:
            return
        dx, dy = event.x - self._view_drag[0], event.y - self._view_drag[1]
        self._view_drag = (event.x, event.y)
        self.yaw += dx * 0.01
        self.pitch = max(0.05, min(math.pi / 2, self.pitch + dy * 0.01))
        self.draw3d()

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

    def faces(self, box, color, stipple=""):
        """The six faces of a box as (depth, screen points, color, stipple)."""
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
                            [n for v in quad for n in v[:2]], color, stipple))
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
            faces += self.faces(w.box, "#5a5a5a" if w.name == "floor" else "#7a7a7a",
                                "" if w.name == "floor" else "gray25")
        for o in a.obstacles:
            faces += self.faces(o.box, KIND_COLOR.get(o.kind, "#8a6a3a"), "gray50")
        hit_names = {n for pair in a.collisions() for n in pair}
        for sol in a.moving_solids():
            faces += self.faces(sol.box,
                                HIT_COLOR if sol.name in hit_names else KIND_COLOR[sol.kind],
                                "gray50" if sol.kind == "carriage" else "")

        if self.explored:
            step = max(1, len(self.explored) // MAX_EXPLORED_DOTS)
            for i, (x, y, z) in enumerate(self.explored):
                if i % step == 0:
                    sx, sy, _ = p(x, y, z + m)
                    v.create_oval(sx - 1, sy - 1, sx + 1, sy + 1,
                                  fill="#6a6a6a", outline="")

        for _, pts, color, stipple in sorted(faces, key=lambda f: -f[0]):
            v.create_polygon(pts, fill=color, outline="#cfcfcf", stipple=stipple)

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

        v.create_text(8, VIEW_H - 8, anchor="sw", fill="#777777",
                      text="drag: rotate   wheel: zoom")


if __name__ == "__main__":
    root = tk.Tk()
    AStarGui(root)
    root.mainloop()

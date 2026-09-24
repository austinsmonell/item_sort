"""GUI for the 3D A* pathfinder, built with tkinter (Python standard library
- nothing to install). Calls plan_path() from astar_core.py - the GUI runs the real
algorithm, it does not reimplement it.

The arena (size and resolution) is defined in astar_core.py, in cm.

Left: a top-down map of the arena. Set the Height (z) slider to choose which
height you are working at, then:
  Box   - drag a rectangle; it spans the z range in the "Box z" fields
  Start / Goal - click; placed at the current height
  Erase - click a box to delete it
Boxes that reach the current height are solid; others show as dashed
outlines. The path is a green line, brighter where it is near the current
height.

Right: a 3D view of the whole arena. Drag to rotate, mouse wheel to zoom.
Heights are stretched by Z_STRETCH so a 35 cm tall arena is readable.

Run with:  python astar_gui.py
"""

import math
import time
import tkinter as tk

from astar_core import (ARENA_X_CM, ARENA_Y_CM, ARENA_Z_CM, RESOLUTION_CM,
                        plan_path)

PX = 3                        # map pixels per cm
MAP_W = int(ARENA_X_CM * PX)
MAP_H = int(ARENA_Y_CM * PX)
VIEW_W = 520
VIEW_H = 480
Z_STRETCH = 2.5               # vertical exaggeration in the 3D view
Z_TOL = 1.0                   # cm: path counts as "at" the current height
MAX_EXPLORED_DOTS = 4000


def snap(v, hi):
    """Round a cm value to the resolution and keep it inside the arena."""
    return min(max(round(v / RESOLUTION_CM) * RESOLUTION_CM, 0.0), hi)


class AStarGui:
    def __init__(self, root):
        self.root = root
        root.title("3D A* Pathfinding")

        self.mode = tk.StringVar(value="box")
        self.z = tk.DoubleVar(value=0.0)
        self.box_z0 = tk.StringVar(value="0")
        self.box_z1 = tk.StringVar(value=f"{ARENA_Z_CM:g}")
        self.diagonals = tk.BooleanVar(value=True)
        self.show_explored = tk.BooleanVar(value=False)
        self.cursor = tk.StringVar(value="")

        self.boxes = []          # (x0, y0, z0, x1, y1, z1) in cm
        self.start = None        # (x, y, z) in cm
        self.goal = None
        self.path = []
        self.explored = set()

        self.yaw = math.radians(35)
        self.pitch = math.radians(55)
        self.zoom = 2.2          # 3D view pixels per cm
        self._view_drag = None
        self._anchor = None
        self._preview = None

        toolbar = tk.Frame(root)
        toolbar.pack(fill="x", padx=8, pady=8)
        for text, value in [("Box", "box"), ("Start", "start"),
                             ("Goal", "goal"), ("Erase", "erase")]:
            tk.Radiobutton(toolbar, text=text, value=value, variable=self.mode,
                           indicatoron=False, width=8).pack(side="left", padx=2)
        tk.Label(toolbar, text="Box z (cm):").pack(side="left", padx=(12, 2))
        tk.Entry(toolbar, textvariable=self.box_z0, width=5).pack(side="left")
        tk.Label(toolbar, text="to").pack(side="left", padx=2)
        tk.Entry(toolbar, textvariable=self.box_z1, width=5).pack(side="left")
        tk.Checkbutton(toolbar, text="Diagonals", variable=self.diagonals
                       ).pack(side="left", padx=(12, 2))
        tk.Checkbutton(toolbar, text="Show explored (3D)",
                       variable=self.show_explored).pack(side="left", padx=2)
        tk.Button(toolbar, text="Run A*", command=self.run).pack(side="left", padx=(12, 2))
        tk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=2)

        zbar = tk.Frame(root)
        zbar.pack(fill="x", padx=8)
        tk.Label(zbar, text="Height z (cm):").pack(side="left")
        tk.Scale(zbar, from_=0, to=ARENA_Z_CM, resolution=RESOLUTION_CM,
                 orient="horizontal", length=360, variable=self.z,
                 command=lambda _: self.draw()).pack(side="left", padx=4)
        tk.Label(zbar, textvariable=self.cursor, width=24, anchor="w"
                 ).pack(side="left", padx=8)

        views = tk.Frame(root)
        views.pack(padx=8)
        self.canvas = tk.Canvas(views, width=MAP_W, height=MAP_H, background="#222222")
        self.canvas.pack(side="left")
        self.view = tk.Canvas(views, width=VIEW_W, height=VIEW_H, background="#161616")
        self.view.pack(side="left", padx=(8, 0))

        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.on_motion)
        self.view.bind("<ButtonPress-1>", self.on_view_press)
        self.view.bind("<B1-Motion>", self.on_view_drag)
        self.view.bind("<MouseWheel>",
                       lambda e: self.zoom_by(1.1 if e.delta > 0 else 1 / 1.1))
        self.view.bind("<Button-4>", lambda e: self.zoom_by(1.1))
        self.view.bind("<Button-5>", lambda e: self.zoom_by(1 / 1.1))

        self.status = tk.StringVar(
            value=f"Arena {ARENA_X_CM:g} x {ARENA_Y_CM:g} x {ARENA_Z_CM:g} cm, "
                  f"{RESOLUTION_CM:g} cm resolution. Drag boxes, set Start and "
                  "Goal, then Run A*.")
        tk.Label(root, textvariable=self.status, anchor="w").pack(fill="x", padx=8, pady=8)

        self.draw()

    # ---- coordinates ----------------------------------------------------

    def cm_at(self, event):
        return (snap(event.x / PX, ARENA_X_CM), snap(event.y / PX, ARENA_Y_CM))

    def box_z_range(self):
        try:
            a, b = float(self.box_z0.get()), float(self.box_z1.get())
        except ValueError:
            return 0.0, ARENA_Z_CM
        return snap(min(a, b), ARENA_Z_CM), snap(max(a, b), ARENA_Z_CM)

    # ---- map editing ----------------------------------------------------

    def invalidate(self):
        self.path = []
        self.explored = set()

    def on_motion(self, event):
        x, y = self.cm_at(event)
        self.cursor.set(f"x {x:g}  y {y:g} cm")

    def on_press(self, event):
        x, y = self.cm_at(event)
        mode = self.mode.get()
        z = snap(self.z.get(), ARENA_Z_CM)
        if mode == "box":
            self._anchor = (x, y)
            return
        if mode == "start":
            self.start = (x, y, z)
        elif mode == "goal":
            self.goal = (x, y, z)
        elif mode == "erase":
            self.erase_at(x, y, z)
        self.invalidate()
        self.draw()

    def on_drag(self, event):
        self.on_motion(event)
        if self.mode.get() != "box" or self._anchor is None:
            return
        x, y = self.cm_at(event)
        ax, ay = self._anchor
        if self._preview:
            self.canvas.delete(self._preview)
        self._preview = self.canvas.create_rectangle(
            ax * PX, ay * PX, x * PX, y * PX, outline="#ffd24a", dash=(4, 3))

    def on_release(self, event):
        if self.mode.get() != "box" or self._anchor is None:
            return
        x, y = self.cm_at(event)
        ax, ay = self._anchor
        self._anchor = None
        self._preview = None
        z0, z1 = self.box_z_range()
        if x != ax and y != ay:
            self.boxes.append((min(ax, x), min(ay, y), z0, max(ax, x), max(ay, y), z1))
        self.invalidate()
        self.draw()

    def erase_at(self, x, y, z):
        """Delete the newest box under (x, y), preferring one at height z."""
        under = [i for i, b in enumerate(self.boxes)
                 if b[0] <= x <= b[3] and b[1] <= y <= b[4]]
        at_z = [i for i in under if self.boxes[i][2] <= z <= self.boxes[i][5]]
        pick = at_z or under
        if pick:
            del self.boxes[pick[-1]]

    def run(self):
        if self.start is None or self.goal is None:
            self.status.set("Set both a Start and a Goal first.")
            return
        self.status.set("Planning...")
        self.root.update_idletasks()
        self.explored = set()
        t = time.time()
        self.path = plan_path(
            self.boxes, self.start, self.goal, diagonals=self.diagonals.get(),
            explored=self.explored if self.show_explored.get() else None)
        dt = time.time() - t
        if self.path:
            length = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
            self.status.set(f"Path found: {len(self.path)} points, {length:.1f} cm "
                            f"({dt:.1f} s).")
        else:
            self.status.set(f"No path found ({dt:.1f} s).")
        self.draw()

    def clear(self):
        self.boxes.clear()
        self.start = self.goal = None
        self.invalidate()
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
        X = x - ARENA_X_CM / 2
        Y = y - ARENA_Y_CM / 2
        Z = (z - ARENA_Z_CM / 2) * Z_STRETCH
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
        zc = snap(self.z.get(), ARENA_Z_CM)

        for x in range(0, int(ARENA_X_CM) + 1, 10):
            c.create_line(x * PX, 0, x * PX, MAP_H, fill="#2f2f2f")
        for y in range(0, int(ARENA_Y_CM) + 1, 10):
            c.create_line(0, y * PX, MAP_W, y * PX, fill="#2f2f2f")

        for (x0, y0, z0, x1, y1, z1) in self.boxes:
            if z0 <= zc <= z1:
                c.create_rectangle(x0 * PX, y0 * PX, x1 * PX, y1 * PX,
                                   fill="#5a5a5a", outline="#8a8a8a")
            else:
                c.create_rectangle(x0 * PX, y0 * PX, x1 * PX, y1 * PX,
                                   outline="#555555", dash=(3, 3))

        if self.path:
            pts = [v for p in self.path for v in (p[0] * PX, p[1] * PX)]
            c.create_line(pts, fill="#2f6b4b", width=1)
            for a, b in zip(self.path, self.path[1:]):
                if abs(a[2] - zc) <= Z_TOL and abs(b[2] - zc) <= Z_TOL:
                    c.create_line(a[0] * PX, a[1] * PX, b[0] * PX, b[1] * PX,
                                  fill="#46b478", width=3)

        for cell, color in ((self.start, "#3c96ff"), (self.goal, "#f05032")):
            if cell:
                x, y, z = cell
                r = 6
                if abs(z - zc) <= Z_TOL:
                    c.create_oval(x * PX - r, y * PX - r, x * PX + r, y * PX + r,
                                  fill=color, outline="white")
                else:
                    c.create_oval(x * PX - r, y * PX - r, x * PX + r, y * PX + r,
                                  outline=color, width=2)

    def draw3d(self):
        v = self.view
        v.delete("all")
        p = self.project
        zc = snap(self.z.get(), ARENA_Z_CM)
        X, Y, Z = ARENA_X_CM, ARENA_Y_CM, ARENA_Z_CM

        def flat(points):
            return [c for pt in points for c in pt[:2]]

        # current height plane
        plane = [p(0, 0, zc), p(X, 0, zc), p(X, Y, zc), p(0, Y, zc)]
        v.create_polygon(flat(plane), fill="#1d3550", outline="#4a7db5", width=2)

        # arena wireframe
        corners = {(i, j, k): p(i * X, j * Y, k * Z)
                   for i in (0, 1) for j in (0, 1) for k in (0, 1)}
        for (i, j, k), a in corners.items():
            for b_key in ((1 - i, j, k), (i, 1 - j, k), (i, j, 1 - k)):
                if b_key > (i, j, k):
                    b = corners[b_key]
                    v.create_line(a[0], a[1], b[0], b[1], fill="#444444")

        if self.explored:
            step = max(1, len(self.explored) // MAX_EXPLORED_DOTS)
            for i, (x, y, z) in enumerate(self.explored):
                if i % step == 0:
                    sx, sy, _ = p(x, y, z)
                    v.create_oval(sx - 1, sy - 1, sx + 1, sy + 1,
                                  fill="#6a6a6a", outline="")

        # boxes: all faces, far to near
        faces = []
        for (x0, y0, z0, x1, y1, z1) in self.boxes:
            q = {(i, j, k): p(x1 if i else x0, y1 if j else y0, z1 if k else z0)
                 for i in (0, 1) for j in (0, 1) for k in (0, 1)}
            for axis in range(3):
                for side in (0, 1):
                    quad = [q[k] for k in q if k[axis] == side]
                    # order the 4 corners around the face
                    cx = sum(c[0] for c in quad) / 4
                    cy = sum(c[1] for c in quad) / 4
                    quad.sort(key=lambda c: math.atan2(c[1] - cy, c[0] - cx))
                    faces.append((sum(c[2] for c in quad) / 4, quad))
        for depth, quad in sorted(faces, key=lambda f: -f[0]):
            v.create_polygon(flat(quad), fill="#9a9a9a", outline="#cfcfcf",
                             stipple="gray50")

        if len(self.path) >= 2:
            v.create_line(flat([p(*q) for q in self.path]), fill="#46b478",
                          width=3, joinstyle="round")

        for cell, color, label in ((self.start, "#3c96ff", "S"),
                                   (self.goal, "#f05032", "G")):
            if cell:
                sx, sy, _ = p(*cell)
                r = 7
                v.create_oval(sx - r, sy - r, sx + r, sy + r, fill=color, outline="white")
                v.create_text(sx, sy, text=label, fill="white",
                              font=("TkDefaultFont", 8, "bold"))

        v.create_text(8, VIEW_H - 8, anchor="sw", fill="#777777",
                      text=f"drag: rotate   wheel: zoom   (height x{Z_STRETCH:g})")


if __name__ == "__main__":
    root = tk.Tk()
    AStarGui(root)
    root.mainloop()

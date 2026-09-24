"""Simple GUI for the 3D A* pathfinder, built with tkinter (Python standard
library - nothing to install). Calls the exact same a_star() function from
astar_core.py that astar.py uses - the GUI runs the real algorithm, it does
not reimplement it.

The grid is shown one layer (z level) at a time; use the Layer slider to
move between layers. Path cells on other layers show as small dots.

The right-hand panel is a 3D view of the whole grid: drag to rotate, mouse
wheel to zoom. Walls are see-through squares, the path is a thick green line
and the layer being edited is highlighted.

Run with:  python astar_gui.py
"""

import math
import tkinter as tk

from astar_core import a_star

CELL = 24
COLS = 30
ROWS = 20
DEPTH = 5

# 3D view
VIEW_W = 520
VIEW_H = 480
ZSPACE = 4  # vertical gap between layers in the 3D view, in cell units


class AStarGui:
    def __init__(self, root):
        self.root = root
        root.title("3D A* Pathfinding")

        self.mode = tk.StringVar(value="wall")
        self.layer = tk.IntVar(value=0)
        self.diagonals = tk.BooleanVar(value=False)
        self.show_explored = tk.BooleanVar(value=False)
        self.yaw = math.radians(35)
        self.pitch = math.radians(55)
        self.zoom = 9.0
        self._drag = None
        self.walls = set()
        self.start = None
        self.goal = None
        self.path = []
        self.explored = set()

        toolbar = tk.Frame(root)
        toolbar.pack(fill="x", padx=8, pady=8)

        for text, value in [("Wall", "wall"), ("Start", "start"),
                             ("Goal", "goal"), ("Erase", "erase")]:
            tk.Radiobutton(
                toolbar, text=text, value=value, variable=self.mode,
                indicatoron=False, width=8
            ).pack(side="left", padx=2)

        tk.Checkbutton(toolbar, text="Diagonals", variable=self.diagonals
                       ).pack(side="left", padx=(20, 2))
        tk.Checkbutton(toolbar, text="Show explored (3D)", variable=self.show_explored,
                       command=self.draw).pack(side="left", padx=(10, 2))
        tk.Button(toolbar, text="Run A*", command=self.run).pack(side="left", padx=(10, 2))
        tk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=2)

        layer_bar = tk.Frame(root)
        layer_bar.pack(fill="x", padx=8)
        tk.Label(layer_bar, text="Layer (z):").pack(side="left")
        tk.Scale(layer_bar, from_=0, to=DEPTH - 1, orient="horizontal",
                 variable=self.layer, command=lambda _: self.draw()
                 ).pack(side="left", padx=4)

        views = tk.Frame(root)
        views.pack(padx=8)
        self.canvas = tk.Canvas(
            views, width=COLS * CELL, height=ROWS * CELL, background="#222222"
        )
        self.canvas.pack(side="left")
        self.view = tk.Canvas(
            views, width=VIEW_W, height=VIEW_H, background="#161616"
        )
        self.view.pack(side="left", padx=(8, 0))
        self.view.bind("<ButtonPress-1>", self.on_view_press)
        self.view.bind("<B1-Motion>", self.on_view_drag)
        self.view.bind("<MouseWheel>", self.on_view_wheel)
        self.view.bind("<Button-4>", lambda e: self.zoom_by(1.1))
        self.view.bind("<Button-5>", lambda e: self.zoom_by(1 / 1.1))

        self.status = tk.StringVar(
            value="Click and drag to draw walls. Set a Start and a Goal, then Run A*."
        )
        tk.Label(root, textvariable=self.status, anchor="w").pack(fill="x", padx=8, pady=8)

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)

        self.draw()

    def cell_from_event(self, event):
        x, y = event.x // CELL, event.y // CELL
        if 0 <= x < COLS and 0 <= y < ROWS:
            return x, y, self.layer.get()
        return None

    def on_view_press(self, event):
        self._drag = (event.x, event.y)

    def on_view_drag(self, event):
        if self._drag is None:
            return
        dx, dy = event.x - self._drag[0], event.y - self._drag[1]
        self._drag = (event.x, event.y)
        self.yaw += dx * 0.01
        self.pitch = max(0.05, min(math.pi / 2, self.pitch + dy * 0.01))
        self.draw3d()

    def on_view_wheel(self, event):
        self.zoom_by(1.1 if event.delta > 0 else 1 / 1.1)

    def zoom_by(self, factor):
        self.zoom = max(2.0, min(40.0, self.zoom * factor))
        self.draw3d()

    def project(self, x, y, z):
        """Grid cell centre -> (screen x, screen y, depth). Larger depth is
        farther from the viewer."""
        X = x + 0.5 - COLS / 2
        Y = y + 0.5 - ROWS / 2
        Z = (z - (DEPTH - 1) / 2) * ZSPACE
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        x1 = X * cy - Y * sy
        y1 = X * sy + Y * cy
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        y2 = y1 * cp - Z * sp
        z2 = y1 * sp + Z * cp
        return (VIEW_W / 2 + x1 * self.zoom, VIEW_H / 2 - z2 * self.zoom, y2)

    def paint_cell(self, cell):
        mode = self.mode.get()
        if mode == "wall":
            if cell == self.start or cell == self.goal:
                return
            self.walls.add(cell)
        elif mode == "erase":
            self.walls.discard(cell)
        elif mode == "start":
            self.walls.discard(cell)
            self.start = cell
        elif mode == "goal":
            self.walls.discard(cell)
            self.goal = cell
        self.path = []
        self.explored = set()
        self.draw()

    def on_click(self, event):
        cell = self.cell_from_event(event)
        if cell:
            self.paint_cell(cell)

    def on_drag(self, event):
        if self.mode.get() in ("start", "goal"):
            return
        cell = self.cell_from_event(event)
        if cell:
            self.paint_cell(cell)

    def run(self):
        if self.start is None or self.goal is None:
            self.status.set("Set both a Start and a Goal first.")
            return
        self.explored = set()
        self.path = a_star(self.walls, COLS, ROWS, DEPTH, self.start, self.goal,
                           self.explored, diagonals=self.diagonals.get())
        if self.path:
            self.status.set(f"Path found: {len(self.path)} cells across {len({p[2] for p in self.path})} layer(s) ({len(self.explored)} explored).")
        else:
            self.status.set(f"No path found ({len(self.explored)} explored).")
        self.draw()

    def clear(self):
        self.walls.clear()
        self.start = None
        self.goal = None
        self.path = []
        self.explored = set()
        self.status.set("Cleared.")
        self.draw()

    def draw(self):
        c = self.canvas
        c.delete("all")

        for i in range(COLS + 1):
            c.create_line(i * CELL, 0, i * CELL, ROWS * CELL, fill="#3a3a3a")
        for j in range(ROWS + 1):
            c.create_line(0, j * CELL, COLS * CELL, j * CELL, fill="#3a3a3a")

        z = self.layer.get()

        def rect(x, y, inset=0, **kw):
            c.create_rectangle(x * CELL + inset, y * CELL + inset,
                               (x + 1) * CELL - inset, (y + 1) * CELL - inset, **kw)

        for (x, y, cz) in self.walls:
            if cz == z:
                rect(x, y, fill="#5a5a5a", outline="#3a3a3a")

        for (x, y, cz) in self.explored:
            if cz == z and (x, y, cz) not in self.walls:
                rect(x, y, fill="#2e2e2e", outline="#3a3a3a")

        for (x, y, cz) in self.path:
            if cz == z:
                rect(x, y, inset=4, fill="#46b478", outline="")
            else:
                rect(x, y, inset=10, fill="#2f6b4b", outline="")

        for cell, color in ((self.start, "#3c96ff"), (self.goal, "#f05032")):
            if cell:
                x, y, cz = cell
                if cz == z:
                    rect(x, y, fill=color, outline="#3a3a3a")
                else:
                    rect(x, y, inset=8, fill="", outline=color, width=2)

        self.draw3d()

    def draw3d(self):
        v = self.view
        v.delete("all")
        cur = self.layer.get()
        p = self.project

        # layer planes; the one being edited is highlighted
        for z in range(DEPTH):
            corners = [p(0, 0, z), p(COLS - 1, 0, z), p(COLS - 1, ROWS - 1, z), p(0, ROWS - 1, z)]
            pts = [c for (sx, sy, _) in corners for c in (sx, sy)]
            active = z == cur
            v.create_polygon(pts, fill="#1d3550" if active else "", outline="#4a7db5" if active else "#3a3a3a",
                             width=2 if active else 1)
            sx, sy, _ = corners[3]
            v.create_text(sx - 6, sy, text=f"z={z}", anchor="e",
                          fill="#8fbfff" if active else "#666666", font=("TkDefaultFont", 8))

        if self.show_explored.get():
            for (x, y, z) in self.explored:
                if (x, y, z) in self.walls:
                    continue
                sx, sy, _ = p(x, y, z)
                v.create_oval(sx - 1, sy - 1, sx + 1, sy + 1, fill="#5a5a5a", outline="")

        # walls, far to near, brightness fades with distance
        size = max(2.0, self.zoom * 0.4)
        walls = sorted((p(*w) + (w,) for w in self.walls), key=lambda t: -t[2])
        for sx, sy, depth, (x, y, z) in walls:
            shade = int(max(90, min(200, 150 - depth * 3)))
            color = f"#{shade:02x}{shade:02x}{shade:02x}"
            v.create_rectangle(sx - size, sy - size, sx + size, sy + size,
                               fill=color, outline="#2a2a2a", stipple="gray50")

        if self.path:
            pts = [c for cell in self.path for c in p(*cell)[:2]]
            if len(pts) >= 4:
                v.create_line(pts, fill="#46b478", width=3, joinstyle="round")
            for cell in self.path:
                sx, sy, _ = p(*cell)
                v.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, fill="#46b478", outline="")

        for cell, color, label in ((self.start, "#3c96ff", "S"), (self.goal, "#f05032", "G")):
            if cell:
                sx, sy, _ = p(*cell)
                r = max(5, self.zoom * 0.5)
                v.create_oval(sx - r, sy - r, sx + r, sy + r, fill=color, outline="white")
                v.create_text(sx, sy, text=label, fill="white", font=("TkDefaultFont", 8, "bold"))

        v.create_text(8, VIEW_H - 8, anchor="sw", fill="#777777",
                      text="drag: rotate   wheel: zoom")


if __name__ == "__main__":
    root = tk.Tk()
    AStarGui(root)
    root.mainloop()

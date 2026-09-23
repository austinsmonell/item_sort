"""Simple GUI for the A* pathfinder, built with tkinter (Python standard
library - nothing to install). Calls the exact same a_star() function from
astar_core.py that astar.py uses - the GUI runs the real algorithm, it does
not reimplement it.

Run with:  python astar_gui.py
"""

import tkinter as tk

from astar_core import a_star

CELL = 24
COLS = 30
ROWS = 20


class AStarGui:
    def __init__(self, root):
        self.root = root
        root.title("A* Pathfinding")

        self.mode = tk.StringVar(value="wall")
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

        tk.Button(toolbar, text="Run A*", command=self.run).pack(side="left", padx=(20, 2))
        tk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=2)

        self.canvas = tk.Canvas(
            root, width=COLS * CELL, height=ROWS * CELL, background="#222222"
        )
        self.canvas.pack(padx=8)

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
            return x, y
        return None

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
        self.path = a_star(self.walls, COLS, ROWS, self.start, self.goal, self.explored)
        if self.path:
            self.status.set(f"Path found: {len(self.path)} cells ({len(self.explored)} explored).")
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

        for (x, y) in self.walls:
            c.create_rectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL,
                                fill="#5a5a5a", outline="#3a3a3a")

        for (x, y) in self.explored:
            if (x, y) in self.walls:
                continue
            c.create_rectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL,
                                fill="#2e2e2e", outline="#3a3a3a")

        for (x, y) in self.path:
            c.create_rectangle(x * CELL + 4, y * CELL + 4, (x + 1) * CELL - 4, (y + 1) * CELL - 4,
                                fill="#46b478", outline="")

        if self.start:
            x, y = self.start
            c.create_rectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL,
                                fill="#3c96ff", outline="#3a3a3a")
        if self.goal:
            x, y = self.goal
            c.create_rectangle(x * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL,
                                fill="#f05032", outline="#3a3a3a")


if __name__ == "__main__":
    root = tk.Tk()
    AStarGui(root)
    root.mainloop()

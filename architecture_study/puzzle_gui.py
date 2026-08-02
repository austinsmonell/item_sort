"""Tk front end: build a layout, pick a box, pick a destination, watch it run.

Canvas interaction has two modes, switched by the *Drag boxes* toggle:

  planning (default)  left-click selects a box, left-click on empty floor or
                      right-click anywhere drops the selected box's goal, and
                      the arrow keys nudge that goal a cell at a time.
  drag                left-drag slides a box around by hand, refusing any
                      position that overlaps or leaves the arena.  Whatever you
                      arrange becomes the new starting layout.

Search runs on a worker thread with a cancel flag, and results come back through
a queue that the Tk thread polls.  Nothing but the main thread touches a widget.
"""

import queue
import threading
import tkinter as tk

from anytime_planner import AnytimePlanner
from board import Board, InvalidLayout
from decomposition import DecompositionPlanner
from layout import LayoutFull
from puzzle_state import PuzzleState

LABEL_FONT = ("Segoe UI", 9)
READOUT_FONT = ("Consolas", 9)

CANVAS_BG = "#eef0f3"
ARENA_FILL = "#ffffff"
ARENA_EDGE = "#5a6472"
GRID_LINE = "#e2e5ea"
PATH_LINE = "#8892a0"
SELECT_EDGE = "#14181f"
ILLEGAL_EDGE = "#d92b2b"
STATUS_FG = "#3d4550"
WARNING_FG = "#b3261e"


class PuzzleGUI:
    """Interactive front end for the A* rectangle-sorting study."""

    def __init__(self, config, board: Board, cost_model, generator):
        self.cfg = config
        self.board = board
        self.cost = cost_model
        self.generator = generator

        self.state = board.initial
        self.home = board.initial      # what "Reset layout" goes back to
        self.selected = 0 if board.count else None
        self.goal = None
        self.plan = None
        self.frame = 0
        self.playing = False

        self._colors = {round(size, 4): config.BOX_TYPE_COLORS.get(name)
                        for name, size in config.BOX_TYPES}
        self._results = queue.Queue()
        self._worker = None
        self._cancel = threading.Event()
        self._anim_job = None
        self._drag = None              # (box, grab offset x, grab offset y)
        self._drag_block = None        # cell the drag is currently refused
        self._pending_plan = None      # improvement that arrived mid-playback
        self._scale = 1.0
        self._origin = (0.0, 0.0)
        self._canvas_h = 1

        self._build()
        self._say("Left-click a box, right-click (or arrow keys) to place its "
                  "goal, then plan the move.")

    # ------------------------------------------------------------------ setup

    def _build(self):
        self.root = tk.Tk()
        self.root.title(self.cfg.WINDOW_TITLE)
        self.root.minsize(1060, 800)

        cw, ch = self.cfg.CANVAS_SIZE
        self.canvas = tk.Canvas(self.root, width=cw, height=ch, bg=CANVAS_BG,
                                highlightthickness=0, takefocus=1)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _e: self._redraw())
        self.canvas.bind("<Button-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Button-3>", self._on_right_click)
        for key, delta in (("Left", (-1, 0)), ("Right", (1, 0)),
                           ("Up", (0, 1)), ("Down", (0, -1))):
            self.canvas.bind(f"<{key}>",
                             lambda _e, d=delta: self._nudge_goal(d, 1))
            self.canvas.bind(f"<Shift-{key}>",
                             lambda _e, d=delta: self._nudge_goal(d, 5))

        panel = tk.Frame(self.root, padx=10, pady=8)
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        self._build_controls(panel)

    def _build_controls(self, panel):
        cfg = self.cfg

        layout = tk.LabelFrame(panel, text="Layout", font=LABEL_FONT, padx=8, pady=6)
        layout.pack(fill=tk.X)
        counts_row = tk.Frame(layout)
        counts_row.pack(fill=tk.X)
        self.count_vars = {}
        for type_name, size in cfg.BOX_TYPES:
            column = tk.Frame(counts_row)
            column.pack(side=tk.LEFT, expand=True, fill=tk.X)
            tk.Label(column, text=f"{type_name}\n{size:.2f} m", font=LABEL_FONT,
                     fg=cfg.BOX_TYPE_COLORS.get(type_name, STATUS_FG)).pack()
            var = tk.IntVar(value=cfg.DEFAULT_COUNTS.get(type_name, 0))
            tk.Spinbox(column, from_=0, to=cfg.MAX_COUNT_PER_TYPE, width=4,
                       font=LABEL_FONT, textvariable=var,
                       justify=tk.CENTER).pack(pady=(2, 0))
            self.count_vars[type_name] = var
        buttons = tk.Frame(layout)
        buttons.pack(fill=tk.X, pady=(6, 0))
        tk.Button(buttons, text="Regenerate", font=LABEL_FONT,
                  command=self._regenerate).pack(side=tk.LEFT, expand=True,
                                                 fill=tk.X)
        tk.Button(buttons, text="Clear floor", font=LABEL_FONT,
                  command=self._clear_boxes).pack(side=tk.LEFT, expand=True,
                                                  fill=tk.X)
        self.var_drag = tk.BooleanVar(value=False)
        tk.Checkbutton(layout, text="Drag boxes with the mouse", font=LABEL_FONT,
                       variable=self.var_drag, command=self._on_drag_toggle,
                       anchor="w").pack(fill=tk.X, pady=(2, 0))

        sel = tk.LabelFrame(panel, text="Selection", font=LABEL_FONT, padx=8, pady=6)
        sel.pack(fill=tk.X, pady=(8, 0))
        self.var_selection = tk.StringVar()
        tk.Label(sel, textvariable=self.var_selection, font=READOUT_FONT,
                 justify=tk.LEFT, anchor="w").pack(fill=tk.X)
        tk.Button(sel, text="Clear goal", font=LABEL_FONT,
                  command=self._clear_goal).pack(fill=tk.X, pady=(6, 0))

        weights = tk.LabelFrame(panel, text="Cost weights", font=LABEL_FONT,
                                padx=8, pady=4)
        weights.pack(fill=tk.X, pady=(8, 0))
        self.s_distance = self._slider(
            weights, "Distance  (cost per metre moved)",
            cfg.DISTANCE_WEIGHT_RANGE, 0.05, cfg.DISTANCE_WEIGHT)
        self.s_regrip = self._slider(
            weights, "Re-grip  (cost per box pick-up)",
            cfg.REGRIP_WEIGHT_RANGE, 0.05, cfg.REGRIP_WEIGHT)

        search = tk.LabelFrame(panel, text="Search", font=LABEL_FONT, padx=8, pady=4)
        search.pack(fill=tk.X, pady=(8, 0))
        self.s_heuristic = self._slider(
            search, "Heuristic weight  (1.0 = optimal)",
            cfg.HEURISTIC_WEIGHT_RANGE, 0.1, cfg.HEURISTIC_WEIGHT)
        self.s_nodes = self._slider(
            search, "Node budget", cfg.MAX_NODES_RANGE, 20_000, cfg.MAX_NODES)
        self.s_grid = self._slider(
            search, "Grid step (m)", cfg.GRID_STEP_RANGE, 0.01, cfg.GRID_STEP,
            command=self._on_grid_change)
        self.btn_plan = tk.Button(search, text="Plan move", font=LABEL_FONT,
                                  command=self._on_plan)
        self.btn_plan.pack(fill=tk.X, pady=(6, 0))

        play = tk.LabelFrame(panel, text="Playback", font=LABEL_FONT, padx=8, pady=4)
        play.pack(fill=tk.X, pady=(8, 0))
        self.s_speed = self._slider(play, "Step interval (ms)", cfg.PLAYBACK_MS_RANGE,
                                    5, cfg.PLAYBACK_MS)
        row = tk.Frame(play)
        row.pack(fill=tk.X, pady=(6, 0))
        self.btn_play = tk.Button(row, text="Play", font=LABEL_FONT, width=7,
                                  command=self._on_play)
        self.btn_play.pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(row, text="Step", font=LABEL_FONT, width=7,
                  command=self._on_step).pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(row, text="Rewind", font=LABEL_FONT, width=7,
                  command=self._on_rewind).pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(play, text="Reset layout", font=LABEL_FONT,
                  command=self._on_reset).pack(fill=tk.X, pady=(4, 0))

        result = tk.LabelFrame(panel, text="Result", font=LABEL_FONT, padx=8, pady=6)
        result.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.var_result = tk.StringVar(value="no plan yet")
        tk.Label(result, textvariable=self.var_result, font=READOUT_FONT,
                 justify=tk.LEFT, anchor="nw").pack(fill=tk.BOTH, expand=True)

        self.var_status = tk.StringVar()
        self.status_label = tk.Label(panel, textvariable=self.var_status,
                                     font=LABEL_FONT, fg=STATUS_FG, wraplength=280,
                                     justify=tk.LEFT, anchor="w")
        self.status_label.pack(fill=tk.X, pady=(8, 0))

    def _slider(self, parent, label, span, resolution, value, command=None):
        scale = tk.Scale(parent, label=label, from_=span[0], to=span[1],
                         resolution=resolution, orient=tk.HORIZONTAL,
                         font=LABEL_FONT, length=270, command=command)
        scale.set(value)
        scale.pack(fill=tk.X)
        return scale

    def run(self):
        self._refresh_selection()
        self.canvas.focus_set()
        self._redraw()
        self.root.mainloop()

    # -------------------------------------------------------------- transform

    def _update_transform(self):
        arena = self.board.arena
        cw = max(self.canvas.winfo_width(), 1)
        ch = max(self.canvas.winfo_height(), 1)
        margin = 26
        scale = min((cw - 2 * margin) / arena.w, (ch - 2 * margin) / arena.h)
        self._scale = max(scale, 1e-6)
        self._origin = ((cw - arena.w * self._scale) / 2.0,
                        (ch - arena.h * self._scale) / 2.0)
        self._canvas_h = ch

    def _px(self, x, y):
        """World metres -> canvas pixels (canvas y grows downward)."""
        arena = self.board.arena
        ox, oy = self._origin
        return (ox + (x - arena.x) * self._scale,
                self._canvas_h - oy - (y - arena.y) * self._scale)

    def _world(self, px, py):
        arena = self.board.arena
        ox, oy = self._origin
        return (arena.x + (px - ox) / self._scale,
                arena.y + (self._canvas_h - oy - py) / self._scale)

    def _color(self, size):
        return self._colors.get(round(size, 4)) or self.cfg.FALLBACK_BOX_COLOR

    # ---------------------------------------------------------------- drawing

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        if c.winfo_width() < 20:
            return
        self._update_transform()
        self._refresh_selection()

        arena = self.board.arena
        x0, y0 = self._px(arena.x, arena.y2)
        x1, y1 = self._px(arena.x2, arena.y)
        c.create_rectangle(x0, y0, x1, y1, fill=ARENA_FILL, outline=ARENA_EDGE,
                           width=2)
        self._draw_grid()
        self._draw_plan_outcome()
        self._draw_path()
        self._draw_goal()
        self._draw_boxes()
        self._draw_drag_block()

    def _draw_grid(self):
        step = self.board.step
        if step * self._scale < 6.0:
            return
        arena = self.board.arena
        for i in range(1, int(round(arena.w / step))):
            x, top = self._px(arena.x + i * step, arena.y2)
            _, bottom = self._px(arena.x, arena.y)
            self.canvas.create_line(x, top, x, bottom, fill=GRID_LINE)
        for j in range(1, int(round(arena.h / step))):
            left, y = self._px(arena.x, arena.y + j * step)
            right, _ = self._px(arena.x2, arena.y)
            self.canvas.create_line(left, y, right, y, fill=GRID_LINE)

    def _draw_boxes(self):
        for i, rect in enumerate(self.board.rects(self.state)):
            x0, y0 = self._px(rect.x, rect.y2)
            x1, y1 = self._px(rect.x2, rect.y)
            selected = i == self.selected
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill=self._color(rect.w),
                outline=SELECT_EDGE if selected else "#2b3038",
                width=3 if selected else 1)
            cx, cy = self._px(*rect.center)
            size = min(x1 - x0, y0 - y1)
            if size > 26:
                self.canvas.create_text(cx, cy, text=self.board.names[i],
                                        fill="#ffffff",
                                        font=("Segoe UI", 10, "bold"))

        held = self.state.last_moved
        if (self.plan is not None and 0 <= held < self.board.count
                and held != self.selected):
            rect = self.board.rect(held, self.state.cells[held])
            x0, y0 = self._px(rect.x, rect.y2)
            x1, y1 = self._px(rect.x2, rect.y)
            self.canvas.create_rectangle(x0, y0, x1, y1, outline=SELECT_EDGE,
                                         width=2, dash=(3, 3))

    def _draw_goal(self):
        if self.goal is None or self.selected is None:
            return
        rect = self.board.rect(self.selected, self.goal)
        x0, y0 = self._px(rect.x, rect.y2)
        x1, y1 = self._px(rect.x2, rect.y)
        color = self._color(rect.w)
        self.canvas.create_rectangle(x0, y0, x1, y1, outline=color, width=2,
                                     dash=(5, 4))
        cx, cy = self._px(*rect.center)
        self.canvas.create_text(cx, cy, text=f"{self.board.names[self.selected]}\ngoal",
                                fill=color, font=("Segoe UI", 8, "bold"),
                                justify=tk.CENTER)

    def _draw_plan_outcome(self):
        """Dashed outline of where every moved box ends up under the plan.

        Drawn while the search is still improving, so the board shows what the
        best plan so far actually achieves rather than just the route.
        """
        if self.plan is None or self.frame != 0 or len(self.plan.states) < 2:
            return
        final = self.plan.states[-1]
        for box in self.plan.boxes_moved:
            if final.cells[box] == self.state.cells[box]:
                continue
            rect = self.board.rect(box, final.cells[box])
            x0, y0 = self._px(rect.x, rect.y2)
            x1, y1 = self._px(rect.x2, rect.y)
            self.canvas.create_rectangle(x0, y0, x1, y1, dash=(3, 3),
                                         outline=self._color(rect.w), width=2)

    def _draw_path(self):
        if self.plan is None or self.selected is None or len(self.plan.states) < 2:
            return
        points = []
        for state in self.plan.states:
            center = self.board.rect(self.selected,
                                     state.cells[self.selected]).center
            point = self._px(*center)
            if not points or point != points[-1]:
                points.extend(point)
        if len(points) >= 4:
            self.canvas.create_line(*points, fill=PATH_LINE, width=2, dash=(6, 4))

    def _draw_drag_block(self):
        if self._drag_block is None or self._drag is None:
            return
        rect = self.board.rect(self._drag[0], self._drag_block)
        x0, y0 = self._px(rect.x, rect.y2)
        x1, y1 = self._px(rect.x2, rect.y)
        self.canvas.create_rectangle(x0, y0, x1, y1, outline=ILLEGAL_EDGE, width=2,
                                     dash=(4, 3))

    # ------------------------------------------------------------ interaction

    def _busy(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def _on_press(self, event):
        self.canvas.focus_set()
        x, y = self._world(event.x, event.y)
        box = self.board.box_at(self.state, x, y)

        if self.var_drag.get():
            if box < 0:
                return
            if self._busy():
                self._warn("cancel the search before rearranging boxes")
                return
            self._stop_playback()
            self.selected = box
            rect = self.board.rect(box, self.state.cells[box])
            self._drag = (box, x - rect.x, y - rect.y)
            self.plan = None
            self.frame = 0
        elif box >= 0:
            self.selected = box
            self.goal = None
            self.plan = None
        elif self.selected is not None:
            self.goal = self.board.snap_center(self.selected, x, y)
            self.plan = None
        self._redraw()

    def _on_motion(self, event):
        if self._drag is None:
            return
        box, grab_x, grab_y = self._drag
        x, y = self._world(event.x, event.y)
        cell = self.board.snap_corner(box, x - grab_x, y - grab_y)
        if cell == self.state.cells[box]:
            self._drag_block = None
        elif self.board.can_place(self.state, box, cell):
            cells = list(self.state.cells)
            cells[box] = cell
            self.state = PuzzleState(tuple(cells), self.state.last_moved)
            self._drag_block = None
        else:
            self._drag_block = cell
        self._redraw()

    def _on_release(self, _event):
        if self._drag is None:
            return
        name = self.board.names[self._drag[0]]
        self._drag = None
        self._drag_block = None
        self.home = self.state
        self._say(f"moved {name} by hand — this is now the starting layout")
        self._redraw()

    def _on_drag_toggle(self):
        if self.var_drag.get():
            self._say("drag mode: left-drag a box to rearrange the floor")
        else:
            self._say("planning mode: left-click selects, right-click sets a goal")

    def _on_right_click(self, event):
        if self.selected is None:
            return
        x, y = self._world(event.x, event.y)
        self.goal = self.board.snap_center(self.selected, x, y)
        self.plan = None
        self._redraw()

    def _nudge_goal(self, delta, span):
        """Arrow keys walk the goal out from wherever the selected box is."""
        if self.selected is None:
            return "break"
        base = self.goal if self.goal is not None else self.state.cells[self.selected]
        cell = self.board.clamp(self.selected,
                                (base[0] + delta[0] * span,
                                 base[1] + delta[1] * span))
        if cell != self.goal:
            self.goal = cell
            self.plan = None
            self._redraw()
        return "break"

    def _clear_goal(self):
        self.goal = None
        self.plan = None
        self._redraw()

    # ----------------------------------------------------------- layout edits

    def _counts(self):
        counts = {}
        for type_name, _ in self.cfg.BOX_TYPES:
            try:
                counts[type_name] = max(0, int(self.count_vars[type_name].get()))
            except (tk.TclError, ValueError):
                counts[type_name] = 0
        return counts

    def _adopt(self, board, message, keep_selection=False):
        self._stop_playback()
        previous = self.selected
        self.board = board
        self.state = board.initial
        self.home = board.initial
        self.goal = None
        self.plan = None
        self.frame = 0
        if keep_selection and previous is not None and previous < board.count:
            self.selected = previous
        else:
            self.selected = 0 if board.count else None
        self.var_result.set("no plan yet")
        self._say(message)
        self._redraw()

    def _regenerate(self):
        if self._busy():
            self._warn("cancel the search before regenerating the layout")
            return
        counts = self._counts()
        step = float(self.s_grid.get())
        try:
            boxes = self.generator.generate(counts, step)
            board = Board(self.cfg.ARENA, boxes, step)
        except (LayoutFull, InvalidLayout) as exc:
            self._warn(str(exc))
            return
        self._adopt(board, f"placed {sum(counts.values())} boxes covering "
                           f"{self.generator.fill_fraction(counts) * 100:.0f}% "
                           "of the floor")

    def _clear_boxes(self):
        if self._busy():
            self._warn("cancel the search before clearing the floor")
            return
        board = Board(self.cfg.ARENA, [], float(self.s_grid.get()))
        self._adopt(board, "floor cleared — set the counts and regenerate to "
                           "put boxes back")

    def _on_grid_change(self, _value=None):
        step = float(self.s_grid.get())
        if abs(step - self.board.step) < 1e-9:
            return
        if self._busy():
            self._warn("cancel the search before changing the grid")
            self.s_grid.set(self.board.step)
            return
        spec = [(self.board.names[i], rect.x, rect.y, rect.w, rect.h)
                for i, rect in enumerate(self.board.rects(self.state))]
        try:
            board = Board(self.cfg.ARENA, spec, step)
        except InvalidLayout:
            self._warn(f"grid step {step:.2f} m rejected — boxes would overlap "
                       "once snapped to it; try a finer step, or regenerate the "
                       "layout on the new grid")
            self.s_grid.set(self.board.step)  # early-out above stops recursion
            return
        self._adopt(board, f"grid step {step:.2f} m — boxes snapped to it",
                    keep_selection=True)

    # --------------------------------------------------------------- planning

    def _on_plan(self):
        if self._busy():
            self._cancel.set()
            self._say("stopping — the best plan found so far is kept…")
            return
        if self.selected is None or self.goal is None:
            self._warn("select a box and place a goal first")
            return

        self._stop_playback()
        self.plan = None               # the old plan is about to be replaced
        self._pending_plan = None
        self.frame = 0
        self.cost.distance_weight = float(self.s_distance.get())
        self.cost.regrip_weight = float(self.s_regrip.get())
        self.cost.heuristic_weight = float(self.s_heuristic.get())

        planner = AnytimePlanner(self.board, self.cost, int(self.s_nodes.get()))
        self._cancel = threading.Event()
        start, target, goal = self.state, self.selected, self.goal

        finder = DecompositionPlanner(self.board, self.cost)

        def work():
            # Decomposition first: it answers "is there a way at all" in about a
            # millisecond, and that plan goes on the board before the optimiser
            # has done anything.  It then seeds the search, so every round is
            # priced against a real plan from its first node.
            seed = finder.plan(start, target, goal)
            if seed is not None:
                self._results.put(("better", seed))
            # Each further improvement is posted the moment it is found.
            planner.plan(start, target, goal, self._cancel,
                         on_improve=lambda plan: self._results.put(
                             ("better", plan)),
                         seed=seed)
            self._results.put(("done", None))

        self._worker = threading.Thread(target=work, daemon=True)
        self.btn_plan.config(text="Stop optimising — keep best")
        self._say(f"looking for any way to move "
                  f"{self.board.names[target]}…")
        self.var_result.set("searching…")
        self._worker.start()
        self.root.after(60, self._poll)

    def _poll(self):
        """Drain everything the worker has posted, newest plan wins."""
        finished = False
        improved = False
        while True:
            try:
                kind, plan = self._results.get_nowait()
            except queue.Empty:
                break
            if kind == "better":
                self._adopt_plan(plan)
                improved = True
            else:
                finished = True

        if improved:
            self._redraw()
        if not finished:
            self.root.after(60, self._poll)
            return

        self.btn_plan.config(text="Plan move")
        if self.plan is None:
            self._warn("no plan found — raise the node budget, coarsen the "
                       "grid, or pick a nearer goal")
            self.var_result.set("no plan")
        elif self._cancel.is_set():
            self._say(f"stopped — keeping the best plan found "
                      f"({len(self.plan.moves)} steps). Press Play.")
        else:
            self._say(f"{self.plan.message} — {len(self.plan.moves)} steps. "
                      "Press Play.")
        self._redraw()

    def _adopt_plan(self, plan):
        """Show a newly improved plan, without disturbing playback in progress."""
        if self.playing:
            # Watching the previous one — let it finish rather than yanking the
            # board out from under it.
            self._pending_plan = plan
            self._say(f"better plan found: cost {plan.cost:.3f} "
                      f"(applies when playback stops)")
            return
        self.plan = plan
        self.frame = 0
        self.state = plan.states[0]
        self._pending_plan = None
        self._show_result(plan)
        quality = ("feasible" if plan.bound == float("inf")
                   else f"within {plan.bound:g}x optimal")
        self._say(f"cost {plan.cost:.3f}, {quality} — still improving; "
                  "Stop to keep this one")

    def _show_result(self, plan):
        if not plan.found:
            self.var_result.set(
                f"no plan\n"
                f"{plan.message}\n\n"
                f"nodes expanded {plan.nodes_expanded:>10,}\n"
                f"edges tried    {plan.nodes_generated:>10,}\n"
                f"elapsed        {plan.elapsed:>10.2f} s")
            return

        names = ", ".join(self.board.names[b] for b in plan.boxes_moved) or "none"
        if plan.bound == float("inf"):
            quality = "feasible — not yet bounded"
        elif plan.bound <= 1.0 + 1e-9:
            quality = "proven optimal"
        else:
            quality = f"within {plan.bound:g}x of optimal"
        lines = [
            f"{quality}",
            "",
            f"steps          {len(plan.moves):>10,}",
            f"boxes moved    {len(plan.boxes_moved):>10}",
            f"  {names}",
            f"re-grips       {plan.regrips:>10}",
            f"total distance {plan.distance:>10.3f} m",
            f"plan cost      {plan.cost:>10.3f}",
            "",
            f"nodes expanded {plan.nodes_expanded:>10,}",
            f"edges tried    {plan.nodes_generated:>10,}",
            f"elapsed        {plan.elapsed:>10.2f} s",
        ]
        if plan.distance_by_box:
            lines.append("")
            lines.append("distance by box")
            for box in plan.boxes_moved:
                lines.append(f"  {self.board.names[box]:<10} "
                             f"{plan.distance_by_box[box]:>8.3f} m")
        self.var_result.set("\n".join(lines))

    # --------------------------------------------------------------- playback

    def _on_play(self):
        if self.plan is None:
            self._warn("nothing to play — plan a move first")
            return
        if self.playing:
            self._stop_playback()
            return
        if self.frame >= len(self.plan.states) - 1:
            self._on_rewind()
        self.playing = True
        self.btn_play.config(text="Pause")
        self._tick()

    def _tick(self):
        if not self.playing or self.plan is None:
            return
        if self.frame >= len(self.plan.states) - 1:
            # The plan has been carried out, so the board is at a new layout.
            # Anything the optimiser is still working on was planned from the
            # *old* one and no longer applies — drop it and stop the search
            # rather than silently rewinding the floor.
            self._stop_playback(adopt=False)
            self._pending_plan = None
            if self._busy():
                self._cancel.set()
            self.home = self.state
            self._say("plan complete — the layout is now the new start")
            return
        self.frame += 1
        self.state = self.plan.states[self.frame]
        self._redraw()
        self._anim_job = self.root.after(int(self.s_speed.get()), self._tick)

    def _stop_playback(self, adopt=True):
        self.playing = False
        if self._anim_job is not None:
            self.root.after_cancel(self._anim_job)
            self._anim_job = None
        self.btn_play.config(text="Play")
        if adopt and self._pending_plan is not None:
            # A better plan turned up while this one was playing, and playback
            # was interrupted rather than finished — the board is still at the
            # layout that plan starts from, so it can be swapped in.
            pending, self._pending_plan = self._pending_plan, None
            self._adopt_plan(pending)
            self._redraw()

    def _on_step(self):
        if self.plan is None:
            return
        self._stop_playback()
        if self.frame < len(self.plan.states) - 1:
            self.frame += 1
            self.state = self.plan.states[self.frame]
            self._redraw()

    def _on_rewind(self):
        if self.plan is None:
            return
        self._stop_playback()
        self.frame = 0
        self.state = self.plan.states[0]
        self._redraw()

    def _on_reset(self):
        self._stop_playback()
        self.state = self.home
        self.plan = None
        self.goal = None
        self.frame = 0
        self.var_result.set("no plan yet")
        self._say("layout reset")
        self._redraw()

    # ----------------------------------------------------------------- status

    def _refresh_selection(self):
        if self.selected is None:
            self.var_selection.set("box   none\ngoal  none")
            return
        rect = self.board.rect(self.selected, self.state.cells[self.selected])
        text = [f"box   {self.board.names[self.selected]}  ({rect.w:.2f} m square)",
                f"at    {rect.x:.2f}, {rect.y:.2f} m"]
        if self.goal is None:
            text.append("goal  none")
        else:
            goal_rect = self.board.rect(self.selected, self.goal)
            text.append(f"goal  {goal_rect.x:.2f}, {goal_rect.y:.2f} m")
        self.var_selection.set("\n".join(text))

    def _say(self, message):
        self.status_label.config(fg=STATUS_FG)
        self.var_status.set(message)

    def _warn(self, message):
        self.status_label.config(fg=WARNING_FG)
        self.var_status.set(message)

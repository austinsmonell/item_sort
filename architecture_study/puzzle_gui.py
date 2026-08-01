"""Tk front end: pick a box, pick a destination, watch the plan run.

The canvas draws the current layout to scale.  Left-click selects a box (or, on
empty floor, drops the goal there); right-click drops the goal anywhere, even on
top of another box — clearing that box out of the way is the planner's job.

Search runs on a worker thread with a cancel flag, and results come back through
a queue that the Tk thread polls.  Nothing but the main thread touches a widget.
"""

import queue
import threading
import tkinter as tk

from astar_planner import AStarPlanner
from board import Board, InvalidLayout

LABEL_FONT = ("Segoe UI", 9)
READOUT_FONT = ("Consolas", 9)

CANVAS_BG = "#eef0f3"
ARENA_FILL = "#ffffff"
ARENA_EDGE = "#5a6472"
GRID_LINE = "#e2e5ea"
PATH_LINE = "#8892a0"
SELECT_EDGE = "#14181f"


class PuzzleGUI:
    """Interactive front end for the A* rectangle-sorting study."""

    def __init__(self, config, board: Board, cost_model):
        self.cfg = config
        self.board = board
        self.cost = cost_model

        self.state = board.initial
        self.selected = 0
        self.goal = None
        self.plan = None
        self.frame = 0
        self.playing = False

        self._results = queue.Queue()
        self._worker = None
        self._cancel = threading.Event()
        self._anim_job = None
        self._scale = 1.0
        self._origin = (0.0, 0.0)
        self._canvas_h = 1

        self._build()
        self._say("Left-click a box to select it. Right-click to drop its goal, "
                  "then plan the move.")

    # ------------------------------------------------------------------ setup

    def _build(self):
        self.root = tk.Tk()
        self.root.title(self.cfg.WINDOW_TITLE)
        self.root.minsize(1020, 620)

        cw, ch = self.cfg.CANVAS_SIZE
        self.canvas = tk.Canvas(self.root, width=cw, height=ch, bg=CANVAS_BG,
                                highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _e: self._redraw())
        self.canvas.bind("<Button-1>", self._on_left_click)
        self.canvas.bind("<Button-3>", self._on_right_click)

        panel = tk.Frame(self.root, padx=10, pady=10)
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        self._build_controls(panel)

    def _build_controls(self, panel):
        cfg = self.cfg

        sel = tk.LabelFrame(panel, text="Selection", font=LABEL_FONT, padx=8, pady=6)
        sel.pack(fill=tk.X)
        self.var_selection = tk.StringVar()
        tk.Label(sel, textvariable=self.var_selection, font=READOUT_FONT,
                 justify=tk.LEFT, anchor="w").pack(fill=tk.X)
        tk.Button(sel, text="Clear goal", font=LABEL_FONT,
                  command=self._clear_goal).pack(fill=tk.X, pady=(6, 0))

        weights = tk.LabelFrame(panel, text="Cost weights", font=LABEL_FONT,
                                padx=8, pady=4)
        weights.pack(fill=tk.X, pady=(10, 0))
        self.s_distance = self._slider(
            weights, "Distance  (cost per metre moved)",
            cfg.DISTANCE_WEIGHT_RANGE, 0.05, cfg.DISTANCE_WEIGHT)
        self.s_regrip = self._slider(
            weights, "Re-grip  (cost per box pick-up)",
            cfg.REGRIP_WEIGHT_RANGE, 0.05, cfg.REGRIP_WEIGHT)

        search = tk.LabelFrame(panel, text="Search", font=LABEL_FONT, padx=8, pady=4)
        search.pack(fill=tk.X, pady=(10, 0))
        self.s_heuristic = self._slider(
            search, "Heuristic weight  (1.0 = optimal)",
            cfg.HEURISTIC_WEIGHT_RANGE, 0.1, cfg.HEURISTIC_WEIGHT)
        self.s_nodes = self._slider(
            search, "Node budget", cfg.MAX_NODES_RANGE, 20_000, cfg.MAX_NODES)
        self.s_grid = self._slider(
            search, "Grid step (m)  — resets the layout",
            cfg.GRID_STEP_RANGE, 0.01, cfg.GRID_STEP, command=self._on_grid_change)
        self.btn_plan = tk.Button(search, text="Plan move", font=LABEL_FONT,
                                  command=self._on_plan)
        self.btn_plan.pack(fill=tk.X, pady=(6, 0))

        play = tk.LabelFrame(panel, text="Playback", font=LABEL_FONT, padx=8, pady=4)
        play.pack(fill=tk.X, pady=(10, 0))
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
        result.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.var_result = tk.StringVar(value="no plan yet")
        tk.Label(result, textvariable=self.var_result, font=READOUT_FONT,
                 justify=tk.LEFT, anchor="nw").pack(fill=tk.BOTH, expand=True)

        self.var_status = tk.StringVar()
        tk.Label(panel, textvariable=self.var_status, font=LABEL_FONT, fg="#3d4550",
                 wraplength=280, justify=tk.LEFT, anchor="w").pack(fill=tk.X,
                                                                   pady=(8, 0))

    def _slider(self, parent, label, span, resolution, value, command=None):
        scale = tk.Scale(parent, label=label, from_=span[0], to=span[1],
                         resolution=resolution, orient=tk.HORIZONTAL,
                         font=LABEL_FONT, length=270, command=command)
        scale.set(value)
        scale.pack(fill=tk.X)
        return scale

    def run(self):
        self._refresh_selection()
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

    # ---------------------------------------------------------------- drawing

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        if self.canvas.winfo_width() < 20:
            return
        self._update_transform()
        self._refresh_selection()

        arena = self.board.arena
        x0, y0 = self._px(arena.x, arena.y2)
        x1, y1 = self._px(arena.x2, arena.y)
        c.create_rectangle(x0, y0, x1, y1, fill=ARENA_FILL, outline=ARENA_EDGE,
                           width=2)
        self._draw_grid()
        self._draw_path()
        self._draw_goal()
        self._draw_boxes()

    def _draw_grid(self):
        step = self.board.step
        if step * self._scale < 6.0:
            return
        arena = self.board.arena
        n_x = int(round(arena.w / step))
        n_y = int(round(arena.h / step))
        for i in range(1, n_x):
            x, top = self._px(arena.x + i * step, arena.y2)
            _, bottom = self._px(arena.x, arena.y)
            self.canvas.create_line(x, top, x, bottom, fill=GRID_LINE)
        for j in range(1, n_y):
            left, y = self._px(arena.x, arena.y + j * step)
            right, _ = self._px(arena.x2, arena.y)
            self.canvas.create_line(left, y, right, y, fill=GRID_LINE)

    def _draw_boxes(self):
        for i, rect in enumerate(self.board.rects(self.state)):
            x0, y0 = self._px(rect.x, rect.y2)
            x1, y1 = self._px(rect.x2, rect.y)
            selected = i == self.selected
            self.canvas.create_rectangle(
                x0, y0, x1, y1,
                fill=self.cfg.BOX_COLORS[i % len(self.cfg.BOX_COLORS)],
                outline=SELECT_EDGE if selected else "#2b3038",
                width=3 if selected else 1)
            cx, cy = self._px(*rect.center)
            self.canvas.create_text(cx, cy, text=self.board.names[i],
                                    fill="#ffffff", font=("Segoe UI", 11, "bold"))

        # Flag the box the mechanism is holding during playback.
        held = self.state.last_moved
        if self.plan is not None and 0 <= held < self.board.count and held != self.selected:
            rect = self.board.rect(held, self.state.cells[held])
            x0, y0 = self._px(rect.x, rect.y2)
            x1, y1 = self._px(rect.x2, rect.y)
            self.canvas.create_rectangle(x0, y0, x1, y1, outline="#14181f", width=2,
                                         dash=(3, 3))

    def _draw_goal(self):
        if self.goal is None or self.selected is None:
            return
        rect = self.board.rect(self.selected, self.goal)
        x0, y0 = self._px(rect.x, rect.y2)
        x1, y1 = self._px(rect.x2, rect.y)
        color = self.cfg.BOX_COLORS[self.selected % len(self.cfg.BOX_COLORS)]
        self.canvas.create_rectangle(x0, y0, x1, y1, outline=color, width=2,
                                     dash=(5, 4))
        cx, cy = self._px(*rect.center)
        self.canvas.create_text(cx, cy, text=f"{self.board.names[self.selected]} goal",
                                fill=color, font=("Segoe UI", 9, "bold"))

    def _draw_path(self):
        if self.plan is None or self.selected is None or len(self.plan.states) < 2:
            return
        points = []
        for state in self.plan.states:
            center = self.board.rect(self.selected, state.cells[self.selected]).center
            point = self._px(*center)
            if not points or point != points[-1]:
                points.extend(point)
        if len(points) >= 4:
            self.canvas.create_line(*points, fill=PATH_LINE, width=2, dash=(6, 4))

    # ------------------------------------------------------------ interaction

    def _on_left_click(self, event):
        x, y = self._world(event.x, event.y)
        box = self.board.box_at(self.state, x, y)
        if box >= 0:
            self.selected = box
            self.goal = None
            self.plan = None
        elif self.selected is not None:
            self.goal = self.board.snap_center(self.selected, x, y)
            self.plan = None
        self._refresh_selection()
        self._redraw()

    def _on_right_click(self, event):
        if self.selected is None:
            return
        x, y = self._world(event.x, event.y)
        self.goal = self.board.snap_center(self.selected, x, y)
        self.plan = None
        self._refresh_selection()
        self._redraw()

    def _clear_goal(self):
        self.goal = None
        self.plan = None
        self._refresh_selection()
        self._redraw()

    def _on_grid_change(self, _value=None):
        step = float(self.s_grid.get())
        if abs(step - self.board.step) < 1e-9:
            return
        try:
            board = Board(self.cfg.ARENA, self.cfg.BOXES, step)
        except InvalidLayout as exc:
            self._say(f"grid step {step:.2f} m rejected — {exc}")
            self.s_grid.set(self.board.step)  # early-out above stops recursion
            return
        self._stop_playback()
        self.board = board
        self.state = board.initial
        self.goal = None
        self.plan = None
        self.frame = 0
        self.var_result.set("no plan yet")
        self._say(f"grid step {step:.2f} m — layout reset")
        self._refresh_selection()
        self._redraw()

    # --------------------------------------------------------------- planning

    def _on_plan(self):
        if self._worker is not None and self._worker.is_alive():
            self._cancel.set()
            self._say("cancelling search…")
            return
        if self.selected is None or self.goal is None:
            self._say("select a box and right-click a goal first")
            return

        self._stop_playback()
        self.cost.distance_weight = float(self.s_distance.get())
        self.cost.regrip_weight = float(self.s_regrip.get())
        self.cost.heuristic_weight = float(self.s_heuristic.get())

        planner = AStarPlanner(self.board, self.cost, int(self.s_nodes.get()))
        self._cancel = threading.Event()
        start, target, goal = self.state, self.selected, self.goal

        def work():
            self._results.put(planner.plan(start, target, goal, self._cancel))

        self._worker = threading.Thread(target=work, daemon=True)
        self.btn_plan.config(text="Cancel search")
        self._say(f"planning a move of {self.board.names[target]}…")
        self.var_result.set("searching…")
        self._worker.start()
        self.root.after(60, self._poll)

    def _poll(self):
        try:
            plan = self._results.get_nowait()
        except queue.Empty:
            self.root.after(60, self._poll)
            return

        self.btn_plan.config(text="Plan move")
        if plan.found:
            self.plan = plan
            self.frame = 0
            self.state = plan.states[0]
            self._say(f"{len(plan.moves)} steps — press Play")
        else:
            self.plan = None
            self._say(plan.message)
        self._show_result(plan)
        self._redraw()

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
        lines = [
            f"steps          {len(plan.moves):>10,}",
            f"boxes moved    {len(plan.boxes_moved):>10}   ({names})",
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
                lines.append(f"  {self.board.names[box]:<12} "
                             f"{plan.distance_by_box[box]:>8.3f} m")
        self.var_result.set("\n".join(lines))

    # --------------------------------------------------------------- playback

    def _on_play(self):
        if self.plan is None:
            self._say("nothing to play — plan a move first")
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
            self._stop_playback()
            self._say("plan complete — the layout is now the new start")
            return
        self.frame += 1
        self.state = self.plan.states[self.frame]
        self._redraw()
        self._anim_job = self.root.after(int(self.s_speed.get()), self._tick)

    def _stop_playback(self):
        self.playing = False
        if self._anim_job is not None:
            self.root.after_cancel(self._anim_job)
            self._anim_job = None
        self.btn_play.config(text="Play")

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
        self.state = self.board.initial
        self.plan = None
        self.goal = None
        self.frame = 0
        self.var_result.set("no plan yet")
        self._say("layout reset")
        self._refresh_selection()
        self._redraw()

    # ----------------------------------------------------------------- status

    def _refresh_selection(self):
        if self.selected is None:
            self.var_selection.set("box   none\ngoal  none")
            return
        rect = self.board.rect(self.selected, self.state.cells[self.selected])
        text = [f"box   {self.board.names[self.selected]}"
                f"  ({rect.w:.2f} x {rect.h:.2f} m)",
                f"at    {rect.x:.2f}, {rect.y:.2f} m"]
        if self.goal is None:
            text.append("goal  none")
        else:
            goal_rect = self.board.rect(self.selected, self.goal)
            text.append(f"goal  {goal_rect.x:.2f}, {goal_rect.y:.2f} m")
        self.var_selection.set("\n".join(text))

    def _say(self, message):
        self.var_status.set(message)

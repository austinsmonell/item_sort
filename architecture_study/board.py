"""The static half of a puzzle: the arena, the box sizes and the move grid.

`Board` owns every rule about what a legal layout is, and it is the only place
that converts between grid cells and metres.  The planner asks it for successor
states and never touches geometry itself; the GUI asks it for rectangles to draw
and never touches cells itself.

Movement model (deliberately simple for this study): any box may be moved one
grid step in any of the four cardinal directions, by an idealised mechanism that
occupies no space of its own.  A move is legal when the box stays inside the
arena and does not overlap another box.  Modelling the mechanism's own swept
geometry is the intended next step, and it belongs here — it is a new test in
`_fits`, not a change to the search.
"""

import math
from typing import Iterator, List, Optional, Sequence, Tuple

from geometry import Rect
from puzzle_state import Cell, PuzzleState

# Four-connected motion.  Diagonals are left out so that every step costs the
# same distance, which keeps the heuristic honest (see CostModel).
DIRECTIONS: Tuple[Cell, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))

# Tolerance for turning a metre position into a cell index.
_SNAP_EPS = 1e-6


class InvalidLayout(ValueError):
    """Raised when a starting layout does not fit the arena or the grid."""


class Board:
    """Arena, box sizes and the grid that box corners are allowed to sit on."""

    def __init__(self, arena: Rect, boxes: Sequence[tuple], step: float):
        """`boxes` is a sequence of (name, x, y, w, h) in metres."""
        if step <= 0.0:
            raise InvalidLayout("grid step must be positive")

        self.arena = arena
        self.step = step
        self.names: List[str] = [b[0] for b in boxes]
        self.sizes: List[Tuple[float, float]] = [(b[3], b[4]) for b in boxes]

        # Highest legal cell index per box, so bounds checks are pure integer
        # comparisons in the search loop.
        self._max_cell: List[Cell] = []
        for name, (w, h) in zip(self.names, self.sizes):
            mx = int(math.floor((arena.w - w) / step + _SNAP_EPS))
            my = int(math.floor((arena.h - h) / step + _SNAP_EPS))
            if mx < 0 or my < 0:
                raise InvalidLayout(f"box {name} ({w} x {h} m) does not fit the arena")
            self._max_cell.append((mx, my))

        self.initial = PuzzleState(
            tuple(self.snap_corner(i, b[1], b[2]) for i, b in enumerate(boxes))
        )
        if not self.is_valid(self.initial):
            raise InvalidLayout(
                f"starting layout is not legal on a {step:.3f} m grid "
                "(boxes overlap once snapped)"
            )

    # ------------------------------------------------------------------ shape

    @property
    def count(self) -> int:
        return len(self.names)

    def rect(self, box: int, cell: Cell) -> Rect:
        """Rectangle occupied by `box` when its corner sits on `cell`."""
        w, h = self.sizes[box]
        return Rect(self.arena.x + cell[0] * self.step,
                    self.arena.y + cell[1] * self.step, w, h)

    def rects(self, state: PuzzleState) -> List[Rect]:
        return [self.rect(i, c) for i, c in enumerate(state.cells)]

    # ------------------------------------------------------------------ rules

    def in_bounds(self, box: int, cell: Cell) -> bool:
        mx, my = self._max_cell[box]
        return 0 <= cell[0] <= mx and 0 <= cell[1] <= my

    def _fits(self, state: PuzzleState, box: int, cell: Cell,
              rects: Optional[List[Rect]] = None) -> bool:
        """True when `box` may occupy `cell` given where everything else is.

        This is the whole legality rule, and the one place a future mechanism
        model (swept volume, approach direction) would hook into.  `rects` lets a
        caller pass the current layout it has already built, which matters
        because the search calls this four times per box per expansion.
        """
        if not self.in_bounds(box, cell):
            return False
        if rects is None:
            rects = self.rects(state)
        candidate = self.rect(box, cell)
        for other in range(self.count):
            if other != box and candidate.overlaps(rects[other]):
                return False
        return True

    def is_valid(self, state: PuzzleState) -> bool:
        rects = self.rects(state)
        return all(self._fits(state, i, c, rects) for i, c in enumerate(state.cells))

    def neighbors(self, state: PuzzleState) -> Iterator[tuple]:
        """Yield (next_state, box, (dx, dy), distance) for every legal step."""
        rects = self.rects(state)
        for box in range(self.count):
            cx, cy = state.cells[box]
            for dx, dy in DIRECTIONS:
                cell = (cx + dx, cy + dy)
                if self._fits(state, box, cell, rects):
                    yield state.with_move(box, dx, dy), box, (dx, dy), self.step

    # ------------------------------------------------------- metres <-> cells

    def snap_corner(self, box: int, x: float, y: float) -> Cell:
        """Nearest legal cell for a lower-left corner given in metres."""
        cx = int(round((x - self.arena.x) / self.step))
        cy = int(round((y - self.arena.y) / self.step))
        mx, my = self._max_cell[box]
        return (min(max(cx, 0), mx), min(max(cy, 0), my))

    def snap_center(self, box: int, px: float, py: float) -> Cell:
        """Nearest legal cell that puts the box's centre closest to a point."""
        w, h = self.sizes[box]
        return self.snap_corner(box, px - w / 2.0, py - h / 2.0)

    def box_at(self, state: PuzzleState, px: float, py: float) -> int:
        """Index of the box under a point in metres, or -1.  Topmost wins."""
        for box in reversed(range(self.count)):
            if self.rect(box, state.cells[box]).contains_point(px, py):
                return box
        return -1

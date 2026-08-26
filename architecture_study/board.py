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
from typing import Iterator, List, Sequence, Tuple

from geometry import EPS, Rect
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

    def __init__(self, arena: Rect, boxes: Sequence[tuple], step: float,
                 gripper=None):
        """`boxes` is a sequence of (name, x, y, w, h) in metres.

        With a `gripper`, a move is legal only if the mechanism can actually
        make it: the jaws travel with the box and must clear everything else,
        and they must be able to open where the box is picked up and set down.
        Without one, a move only has to keep the box inside the arena and off
        the other boxes — the idealised model the study started from.
        """
        if step <= 0.0:
            raise InvalidLayout("grid step must be positive")

        self.arena = arena
        self.step = step
        self.gripper = gripper
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

        # Configuration-space overlap ranges.  Box i at cell (cx, cy) overlaps
        # box j at (jx, jy) exactly when
        #     jx + back_x[i] <= cx <= jx + fwd_x[j]   (and the same in y).
        # Both offsets fall straight out of the float overlap test and depend
        # only on the two box widths, never on where either box is — so the hot
        # loop is four integer comparisons per pair instead of two rectangles
        # built and four float comparisons.  `_verify_cspace` proves the two
        # agree.
        self._back_x = [int(math.floor(-(w - EPS) / step)) + 1 for w, _ in self.sizes]
        self._back_y = [int(math.floor(-(h - EPS) / step)) + 1 for _, h in self.sizes]
        self._fwd_x = [int(math.ceil((w - EPS) / step)) - 1 for w, _ in self.sizes]
        self._fwd_y = [int(math.ceil((h - EPS) / step)) - 1 for _, h in self.sizes]
        self._verify_cspace()

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

    def max_cell(self, box: int) -> Cell:
        """Highest legal cell index for `box`, ignoring the other boxes."""
        return self._max_cell[box]

    def clamp(self, box: int, cell: Cell) -> Cell:
        """Nearest cell to `cell` that keeps `box` inside the arena."""
        mx, my = self._max_cell[box]
        return (min(max(cell[0], 0), mx), min(max(cell[1], 0), my))

    # ------------------------------------------------------------------ rules

    def in_bounds(self, box: int, cell: Cell) -> bool:
        mx, my = self._max_cell[box]
        return 0 <= cell[0] <= mx and 0 <= cell[1] <= my

    def boxes_overlap(self, i: int, ci: Cell, j: int, cj: Cell) -> bool:
        """True when box `i` at cell `ci` would overlap box `j` at cell `cj`.

        Integer-only equivalent of `self.rect(i, ci).overlaps(self.rect(j, cj))`.
        """
        cx, cy = ci
        jx, jy = cj
        return (jx + self._back_x[i] <= cx <= jx + self._fwd_x[j]
                and jy + self._back_y[i] <= cy <= jy + self._fwd_y[j])

    def overlap_range(self, i: int, j: int, cj: Cell) -> Tuple[int, int, int, int]:
        """Inclusive cell range (lo_x, hi_x, lo_y, hi_y) in which box `i` would
        overlap box `j` sitting at `cj`.

        The rectangular shadow one box casts in another's configuration space.
        `boxes_overlap` answers this a cell at a time; this hands over the whole
        rectangle, which is what lets the decomposition planner paint a blocked
        region in one go.
        """
        return (cj[0] + self._back_x[i], cj[0] + self._fwd_x[j],
                cj[1] + self._back_y[i], cj[1] + self._fwd_y[j])

    def can_place(self, state: PuzzleState, box: int, cell: Cell) -> bool:
        """True when `box` may occupy `cell` given where everything else is.

        This is the whole legality rule, and the one place a future mechanism
        model (swept volume, approach direction, stand-off clearance) would hook
        into.  It is also the hottest function in the study — the search calls it
        four times per box per expansion — hence the inlined integer form.
        """
        cx, cy = cell
        mx, my = self._max_cell[box]
        if not (0 <= cx <= mx and 0 <= cy <= my):
            return False
        back_x = self._back_x[box]
        back_y = self._back_y[box]
        fwd_x = self._fwd_x
        fwd_y = self._fwd_y
        for j, (jx, jy) in enumerate(state.cells):
            if j == box:
                continue
            if (jx + back_x <= cx <= jx + fwd_x[j]
                    and jy + back_y <= cy <= jy + fwd_y[j]):
                return False
        return True

    def can_carry(self, state: PuzzleState, box: int, cell: Cell,
                  diagonal: int) -> bool:
        """True when the *gripped* box may occupy `cell`, jaws and all.

        The jaws move with the box, so the body that has to fit is the box plus
        its open jaws — which is why a carried box cannot always follow a gap
        the bare box would slide through.
        """
        if not self.can_place(state, box, cell):
            return False
        if self.gripper is None:
            return True
        held = self.rect(box, cell)
        others = [self.rect(j, c) for j, c in enumerate(state.cells) if j != box]
        for part in self.gripper.open_jaws(held, diagonal):
            if not self.arena.contains(part):
                return False
            for other in others:
                if part.overlaps(other):
                    return False
        return True

    def carry_options(self, state: PuzzleState, box: int, cell: Cell,
                      current: int) -> Tuple[int, ...]:
        """Which diagonals could hold `box` at `cell`.

        A box already in the jaws keeps the diagonal it is held on — the
        mechanism would have to put it down to change grip.  A box being picked
        up may be taken either way round.
        """
        if self.gripper is None:
            return (-1,)
        wanted = (current,) if current >= 0 else (0, 1)
        return tuple(d for d in wanted if self.can_carry(state, box, cell, d))

    def carry_path(self, state: PuzzleState, box: int,
                   path: Sequence[Cell]):
        """Walk `box` along `path` in one grip, or return None.

        One grip means one diagonal for the whole leg, so the diagonal is chosen
        once and has to survive every step of it.  Returns the states passed
        through, the first being `state` itself.
        """
        options = (0, 1) if self.gripper is not None else (-1,)
        for diagonal in options:
            walked: List[PuzzleState] = [state]
            current = state
            for previous, cell in zip(path, path[1:]):
                if cell == previous:
                    continue
                if not self.can_carry(current, box, cell, diagonal):
                    break
                current = current.with_move(box, cell[0] - previous[0],
                                            cell[1] - previous[1], diagonal)
                walked.append(current)
            else:
                return walked
        return None

    def is_valid(self, state: PuzzleState) -> bool:
        return all(self.can_place(state,i, c) for i, c in enumerate(state.cells))

    def neighbors(self, state: PuzzleState) -> Iterator[tuple]:
        """Yield (next_state, box, (dx, dy), distance) for every legal step."""
        step = self.step
        for box in range(self.count):
            cx, cy = state.cells[box]
            # Keeping hold of the same box keeps the same grip; taking a new one
            # is a fresh pick, and either diagonal is available for it.
            holding = state.held if box == state.last_moved else -1
            for dx, dy in DIRECTIONS:
                cell = (cx + dx, cy + dy)
                if not self.can_place(state, box, cell):
                    continue
                for diagonal in self.carry_options(state, box, cell, holding):
                    yield (state.with_move(box, dx, dy, diagonal), box,
                           (dx, dy), step)

    def _verify_cspace(self):
        """Assert the integer overlap ranges agree with the float geometry.

        The ranges are an optimisation of `Rect.overlaps`, and a slip in the
        derivation would not crash — it would quietly emit plans that overlap
        boxes.  Translation invariance means one reference position per pair
        covers every case, so the whole check is a few thousand comparisons at
        construction.
        """
        base = (5, 5)
        for i in range(self.count):
            for j in range(self.count):
                if i == j:
                    continue
                rj = self.rect(j, base)
                for cx in range(base[0] + self._back_x[i] - 1,
                                base[0] + self._fwd_x[j] + 2):
                    for cy in range(base[1] + self._back_y[i] - 1,
                                    base[1] + self._fwd_y[j] + 2):
                        if (self.boxes_overlap(i, (cx, cy), j, base)
                                != self.rect(i, (cx, cy)).overlaps(rj)):
                            raise InvalidLayout(
                                f"c-space range for boxes {i}/{j} disagrees with "
                                f"the rectangle test at cell {(cx, cy)}")

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

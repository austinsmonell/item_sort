"""The dynamic half of a puzzle: where every box currently sits.

A state holds only what changes during a search.  Box sizes, names and the arena
are static and live in `Board`, so the search can copy states cheaply and hash
them as plain tuples of small integers.

`last_moved` is part of the state on purpose.  The cost function charges for
*picking a box up*, so "box 2 is already in the gripper" is information the
search needs in order to keep two otherwise identical layouts apart.

A NamedTuple, not a dataclass: A* hashes and compares hundreds of thousands of
these, and tuple hashing is done in C.
"""

from typing import NamedTuple, Tuple

Cell = Tuple[int, int]


class PuzzleState(NamedTuple):
    """Grid cell of every box's lower-left corner, plus the box last moved."""

    cells: Tuple[Cell, ...]
    last_moved: int = -1  # -1 = nothing has been moved yet

    def with_move(self, box: int, dx: int, dy: int) -> "PuzzleState":
        """Return a new state with `box` shifted by one grid step."""
        cells = list(self.cells)
        cx, cy = cells[box]
        cells[box] = (cx + dx, cy + dy)
        return PuzzleState(tuple(cells), box)

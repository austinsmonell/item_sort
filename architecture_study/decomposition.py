"""Find *a* way to move a box, fast, by asking a much smaller question.

A* searches the joint space of every box position at once.  That is what makes it
optimal and what makes it hopeless on a crowded floor: a goal that takes 19 nodes
with 24 boxes cannot be reached at all with 32, and no amount of heuristic
weighting rescues it — measured, an obstruction-counting heuristic bought 1.4x on
boards that already worked and nothing on the ones that did not.

So this asks a different question, the one from Stilman & Kuffner's work on
navigation among movable obstacles:

  1. Route the target to its goal as if the other boxes were not there.  One
     box, a few hundred cells, breadth-first — microseconds.
  2. See which boxes that route runs through.
  3. Shove each of them clear of the whole route.  A box that is itself hemmed
     in is the same problem one level down: work out what is pinning it, clear
     that first, then come back.
  4. If some box simply cannot be cleared, make the target treat it as a wall
     and route again.

Cost tracks the number of *obstructions* — usually two to five — rather than the
number of boxes on the floor, which is why it survives a density that stops A*
outright.

This is deliberately only a feasibility finder.  It does not try to be cheap, it
tries to be quick and to succeed; `AnytimePlanner` takes the plan it produces as
a starting incumbent and improves it against the real cost function, with a
bound.  That split is the point: this answers "is there a way, and what is it",
and the optimiser answers "how good can it get".

What it will not find: plans needing a box shoved twice, or needing the target to
move partway before a box can be cleared.  Every displaced box moves once, and
the target goes last.
"""

from collections import deque
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

from astar_planner import Move, Plan
from board import DIRECTIONS, Board
from cost_model import CostModel
from puzzle_state import Cell, PuzzleState


class CellMask:
    """Boolean grid over every cell one particular box could occupy."""

    __slots__ = ("bits", "width", "max_x", "max_y")

    def __init__(self, board: Board, box: int):
        self.max_x, self.max_y = board.max_cell(box)
        self.width = self.max_x + 1
        self.bits = bytearray(self.width * (self.max_y + 1))

    def blocked(self, cell: Cell) -> bool:
        cx, cy = cell
        if not (0 <= cx <= self.max_x and 0 <= cy <= self.max_y):
            return True
        return self.bits[cy * self.width + cx] != 0

    def merge(self, other: "CellMask"):
        """Add another mask's blocked cells to this one."""
        bits, incoming = self.bits, other.bits
        for i, value in enumerate(incoming):
            if value:
                bits[i] = 1

    def block_box(self, board: Board, box: int, other: int, cell: Cell):
        """Block every position of `box` that would overlap `other` at `cell`."""
        lo_x, hi_x, lo_y, hi_y = board.overlap_range(box, other, cell)
        lo_x = max(lo_x, 0)
        hi_x = min(hi_x, self.max_x)
        lo_y = max(lo_y, 0)
        hi_y = min(hi_y, self.max_y)
        if lo_x > hi_x or lo_y > hi_y:
            return
        run = b"\x01" * (hi_x - lo_x + 1)
        for cy in range(lo_y, hi_y + 1):
            start = cy * self.width + lo_x
            self.bits[start:start + len(run)] = run


def _bfs(box: int, start: Cell, mask: CellMask,
         accept: Callable[[Cell], bool]) -> Optional[List[Cell]]:
    """Shortest route for one box to the first cell `accept` likes.

    Every step costs the same, so breadth-first already gives the shortest.
    Returns the cells travelled, `start` included, or None.
    """
    if mask.blocked(start):
        return None
    if accept(start):
        return [start]

    max_x, max_y = mask.max_x, mask.max_y
    came: Dict[Cell, Optional[Cell]] = {start: None}
    queue = deque([start])
    while queue:
        cx, cy = queue.popleft()
        for dx, dy in DIRECTIONS:
            nxt = (cx + dx, cy + dy)
            if nxt in came:
                continue
            if not (0 <= nxt[0] <= max_x and 0 <= nxt[1] <= max_y):
                continue
            if mask.blocked(nxt):
                continue
            came[nxt] = (cx, cy)
            if accept(nxt):
                path: List[Cell] = []
                node: Optional[Cell] = nxt
                while node is not None:
                    path.append(node)
                    node = came[node]
                path.reverse()
                return path
            queue.append(nxt)
    return None


class DecompositionPlanner:
    """Feasibility first: route the target, then clear whatever is in the way."""

    def __init__(self, board: Board, cost_model: CostModel,
                 max_reroutes: int = 8, max_clears: int = 60):
        self.board = board
        self.cost = cost_model
        self.max_reroutes = max_reroutes
        self.max_clears = max_clears

    # ------------------------------------------------------------------ entry

    def plan(self, start: PuzzleState, target: int, goal: Cell) -> Optional[Plan]:
        """A legal plan, or None.  Never raises, never returns anything illegal."""
        board = self.board
        if not board.in_bounds(target, goal):
            return None
        if start.cells[target] == goal:
            return Plan(True, "box is already on the goal", states=[start])

        walls: Set[int] = set()          # boxes the target must route around
        for _ in range(self.max_reroutes):
            route = self._route(start, target, goal, walls)
            if route is None:
                return None              # not even reachable through the walls

            blockers = self._blockers(start, target, route, walls)
            if not blockers:
                displaced: List[Tuple[int, List[Cell]]] = []
            else:
                displaced = self._clear(start, target, route, blockers)
                if displaced is None:
                    # Something in the way refuses to budge.  Treat the first
                    # such box as scenery and find the target another way round.
                    walls.add(blockers[0])
                    continue

            built = self._assemble(start, target, route, displaced)
            if built is not None:
                return built
            walls.add(blockers[0] if blockers else target)
        return None

    # ---------------------------------------------------------------- routing

    def _route(self, start: PuzzleState, target: int, goal: Cell,
               walls: Set[int]) -> Optional[List[Cell]]:
        """The target's shortest route, movable boxes ignored, `walls` solid."""
        mask = CellMask(self.board, target)
        for other in walls:
            mask.block_box(self.board, target, other, start.cells[other])
        return _bfs(target, start.cells[target], mask, lambda cell: cell == goal)

    def _blockers(self, start: PuzzleState, target: int, route: Sequence[Cell],
                  walls: Set[int]) -> List[int]:
        """Boxes the route runs through, in the order the target meets them."""
        board = self.board
        seen: List[int] = []
        for cell in route:
            for other in range(board.count):
                if other == target or other in walls or other in seen:
                    continue
                if board.boxes_overlap(target, cell, other, start.cells[other]):
                    seen.append(other)
        return seen

    # --------------------------------------------------------------- clearing

    def _clear(self, start: PuzzleState, target: int, route: Sequence[Cell],
               blockers: Sequence[int]):
        """Shove everything out of the way.  Returns ordered moves, or None.

        Each box carries its own *keep-out* region — the cells it must not end
        on.  A box on the target's route has to clear the target's corridor; a
        box pinning one of those has to clear *that* box's escape route, which
        is a different region entirely.  Getting this wrong is subtle: a box
        already outside the target's corridor looks "done" even while it is the
        very thing blocking its neighbour, and the chase then runs out of
        candidates and gives up.

        `where` tracks the layout as it goes, so each escape is planned against
        the floor it will actually meet when the moves are run in this order.  A
        box may be asked to move more than once if a later constraint arrives.
        """
        board = self.board
        where: Dict[int, Cell] = dict(enumerate(start.cells))
        displaced: List[Tuple[int, List[Cell]]] = []
        keep_out: Dict[int, CellMask] = {}
        pending: List[int] = []

        def require(box: int, region: CellMask):
            existing = keep_out.get(box)
            if existing is None:
                keep_out[box] = region
            else:
                existing.merge(region)
            if box not in pending:
                pending.insert(0, box)

        for blocker in blockers:
            corridor = CellMask(board, blocker)
            for cell in route:
                corridor.block_box(board, blocker, target, cell)
            require(blocker, corridor)

        for _ in range(self.max_clears):
            if not pending:
                return displaced
            box = pending[0]
            if box == target:
                pending.pop(0)
                continue

            banned = keep_out[box]
            solid = CellMask(board, box)
            for other, cell in where.items():
                if other != box:
                    solid.block_box(board, box, other, cell)

            escape = _bfs(box, where[box], solid,
                          lambda cell: not banned.blocked(cell))
            if escape is not None:
                pending.pop(0)
                if len(escape) > 1:
                    where[box] = escape[-1]
                    displaced.append((box, escape))
                continue

            # Hemmed in.  Work out the route it *would* have taken with the
            # other boxes gone, and make whatever sits on that route clear it.
            through = _bfs(box, where[box], CellMask(board, box),
                           lambda cell: not banned.blocked(cell))
            if through is None:
                return None          # walled in by the arena, not by a box

            pin = None
            for cell in through:
                for other, at in where.items():
                    if other in (box, target):
                        continue
                    if board.boxes_overlap(box, cell, other, at):
                        pin = other
                        break
                if pin is not None:
                    break
            if pin is None:
                return None

            # What the pin has to vacate is this box's route, not the target's.
            region = CellMask(board, pin)
            for cell in through:
                region.block_box(board, pin, box, cell)
            require(pin, region)
        return None

    # --------------------------------------------------------------- assembly

    def _assemble(self, start: PuzzleState, target: int, route: Sequence[Cell],
                  displaced: Sequence[Tuple[int, List[Cell]]]) -> Optional[Plan]:
        """Replay as single-cell moves, checking every layout against the rules.

        Nothing above is trusted: if the schedule does not survive being executed
        in order, this returns None rather than handing back an illegal plan.
        """
        board = self.board
        state = start
        moves: List[Move] = []
        states: List[PuzzleState] = [start]

        legs = [(box, path) for box, path in displaced]
        legs.append((target, list(route)))
        for box, path in legs:
            for previous, cell in zip(path, path[1:]):
                if not board.can_place(state, box, cell):
                    return None
                delta = (cell[0] - previous[0], cell[1] - previous[1])
                state = state.with_move(box, delta[0], delta[1])
                moves.append(Move(box, delta, board.step))
                states.append(state)

        if state.cells[target] != route[-1]:
            return None

        distance_by_box: Dict[int, float] = {}
        regrips = 0
        total = 0.0
        previous_box = start.last_moved
        for move in moves:
            regrip = move.box != previous_box
            regrips += int(regrip)
            total += self.cost.move_cost(move.distance, regrip)
            distance_by_box[move.box] = (distance_by_box.get(move.box, 0.0)
                                         + move.distance)
            previous_box = move.box

        return Plan(
            found=True,
            message=f"feasible plan, {len(displaced)} box(es) moved aside",
            moves=moves,
            states=states,
            cost=total,
            distance=sum(distance_by_box.values()),
            regrips=regrips,
            distance_by_box=distance_by_box,
            nodes_expanded=0,
            nodes_generated=0,
            elapsed=0.0,
            bound=float("inf"),     # quick, not cheap: no optimality claim
        )

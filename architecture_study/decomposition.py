"""Find *a* way to move a box, fast, by driving it and clearing as it goes.

A* searches the joint space of every box position at once.  That is what makes it
optimal and what makes it hopeless on a crowded floor — and it is not a heuristic
problem: an obstruction-counting heuristic bought 1.4x on boards that already
worked and nothing at all on the ones that did not.

So this asks a smaller question, in the spirit of Stilman & Kuffner's work on
navigation among movable obstacles.  The important part is *when* it asks:

    route      work out where the target would go if the other boxes were not
               there — one box, a few hundred cells, breadth-first.
    drive      move it along that route as far as it legally can *right now*.
    clear      it is now nose-to-nose with one box.  Shove that box out of the
               way of the route the target has *left to drive*.
    repeat     drive on.  Re-evaluate after every shove.

Driving before clearing is what makes it robust, and it is worth being precise
about why.  Clearing the whole corridor up front demands that every box in the
target's way find a home outside the *entire* route, which on a packed floor
often has no solution at all.  Clearing as it goes asks for much less:

  * the keep-out region shrinks every time the target advances, so boxes met
    late need to clear almost nothing;
  * cells the target has already driven past are free to park in — a box can be
    shoved into the space right behind it;
  * the target physically moves between shoves, so the floor a box is escaping
    into is the real one at that moment, not a worst-case snapshot.

A box may be shoved more than once — if it gets in the way again later, it is
simply cleared again.  A box that is itself hemmed in is the same problem one
level down: work out what is pinning it, clear that first, come back.  A box that
will not budge at all becomes scenery, and the target routes around it.

Nothing is trusted on the way out: every single-cell move is checked against
`Board.can_place` as the plan is built, and anything that does not survive that
is discarded rather than returned.

This is deliberately a feasibility finder, not an optimiser.  `AnytimePlanner`
takes its plan as a starting incumbent and improves it against the real cost
function, with a bound.  That split is the point: this answers "is there a way,
and what is it", the optimiser answers "how good can it get".
"""

from collections import deque
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

from astar_planner import Move, Plan
from board import DIRECTIONS, Board
from cost_model import CostModel
from puzzle_state import Cell, PuzzleState

# A leg of the finished plan: one box, and the cells it walks through.
Leg = Tuple[int, List[Cell]]


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

    Every step costs the same, so breadth-first already gives the shortest one.
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
    """Drive the target, clear what stops it, repeat."""

    def __init__(self, board: Board, cost_model: CostModel,
                 max_reroutes: int = 8, max_clears: int = 120,
                 short_horizon: int = 6):
        self.board = board
        self.cost = cost_model
        self.max_reroutes = max_reroutes
        self.max_clears = max_clears
        # When a box cannot clear the whole remaining route, it is asked to
        # clear only the next few cells instead — enough for the target to get
        # past it.  If it is in the way again later, it gets shoved again.
        self.short_horizon = short_horizon

    # ------------------------------------------------------------------ entry

    def plan(self, start: PuzzleState, target: int, goal: Cell) -> Optional[Plan]:
        """A legal plan, or None.  Never raises, never returns anything illegal."""
        board = self.board
        if not board.in_bounds(target, goal):
            return None
        if start.cells[target] == goal:
            return Plan(True, "box is already on the goal", states=[start])

        # Two strategies, and which wins is not predictable from the layout:
        # clearing up front is cheaper when it works, because the target drives
        # in a single leg, but the placements it commits to can box in whatever
        # is met later.  Each takes milliseconds, so run both — separately, so
        # that one strategy's dead end does not send the other down the wrong
        # reroute — and keep the cheaper plan.
        best: Optional[Plan] = None
        for up_front in (True, False):
            found = self._attempt(start, target, goal, up_front)
            if found is not None and (best is None or found.cost < best.cost):
                best = found
        return best

    def _attempt(self, start: PuzzleState, target: int, goal: Cell,
                 up_front: bool) -> Optional[Plan]:
        """Drive to the goal, walling off boxes that refuse to move, until it
        works or the reroutes run out."""
        walls: Set[int] = set()
        for _ in range(self.max_reroutes):
            legs, stuck = self._drive(start, target, goal, walls, up_front)
            if legs is not None:
                built = self._assemble(start, target, legs)
                if built is not None:
                    return built
            if stuck is None:
                return None              # no route at all, or nothing to blame
            walls.add(stuck)
        return None

    # ---------------------------------------------------------------- driving

    def _drive(self, start: PuzzleState, target: int, goal: Cell,
               walls: Set[int],
               up_front: bool) -> Tuple[Optional[List[Leg]], Optional[int]]:
        """Walk the target to its goal, clearing whatever stops it on the way.

        Returns (legs, None) on success, or (None, box to route around).
        """
        board = self.board
        route = self._route(start, target, goal, walls)
        if route is None:
            return None, None

        state = start
        legs: List[Leg] = []
        at = 0

        # First try to clear the whole route before setting off.  When that
        # works the target drives it in one leg — one grip instead of one per
        # obstruction, which is much cheaper.  Boxes that will not clear that
        # much are simply left; the loop below deals with them as it meets them,
        # by which point the region they must vacate is far smaller.
        if up_front:
            for blocker in self._on_route(state, target, route):
                cleared = self._clear(state, target, route, blocker)
                if cleared is not None:
                    early, state = cleared
                    legs.extend(early)

        for _ in range(self.max_clears):
            # Drive as far as the floor allows from where we actually are.
            ahead = at
            while (ahead + 1 < len(route)
                   and board.can_place(state, target, route[ahead + 1])):
                ahead += 1
            if ahead > at:
                path = list(route[at:ahead + 1])
                state = self._walk(state, target, path)
                if state is None:
                    return None, None
                legs.append((target, path))
                at = ahead
            if at == len(route) - 1:
                return legs, None

            # Nose to nose with something.  Whatever is sitting on the next cell
            # has to give way — but only over the route still to be driven.
            nxt = route[at + 1]
            blocker = next(
                (other for other in range(board.count)
                 if other != target
                 and board.boxes_overlap(target, nxt, other, state.cells[other])),
                None)
            if blocker is None:
                return None, None        # stopped by the arena, not a box

            cleared = self._clear(state, target, route[at:], blocker)
            if cleared is None:
                return None, blocker
            moved_legs, state = cleared
            legs.extend(moved_legs)
        return None, None

    def _on_route(self, state: PuzzleState, target: int,
                  route: Sequence[Cell]) -> List[int]:
        """Boxes the route runs through, in the order the target meets them."""
        board = self.board
        met: List[int] = []
        for cell in route:
            for other in range(board.count):
                if other == target or other in met:
                    continue
                if board.boxes_overlap(target, cell, other, state.cells[other]):
                    met.append(other)
        return met

    def _route(self, state: PuzzleState, target: int, goal: Cell,
               walls: Set[int]) -> Optional[List[Cell]]:
        """The target's shortest route, movable boxes ignored, `walls` solid."""
        mask = CellMask(self.board, target)
        for other in walls:
            mask.block_box(self.board, target, other, state.cells[other])
        return _bfs(target, state.cells[target], mask, lambda cell: cell == goal)

    def _walk(self, state: PuzzleState, box: int,
              path: Sequence[Cell]) -> Optional[PuzzleState]:
        """Apply a box's path one cell at a time, refusing anything illegal."""
        for previous, cell in zip(path, path[1:]):
            if not self.board.can_place(state, box, cell):
                return None
            state = state.with_move(box, cell[0] - previous[0],
                                    cell[1] - previous[1])
        return state

    # --------------------------------------------------------------- clearing

    def _clear(self, state: PuzzleState, target: int, remaining: Sequence[Cell],
               blocker: int) -> Optional[Tuple[List[Leg], PuzzleState]]:
        """Get `blocker` out of the target's way, moving others if it is pinned.

        `remaining` is the route the target has left, starting from where it
        stands.  The blocker is asked to clear all of it; if it cannot, it is
        asked to clear only the next few cells, which is enough to let the
        target past and is very often possible when the full clearance is not.
        """
        board = self.board
        legs: List[Leg] = []
        keep_out: Dict[int, CellMask] = {}
        attempted: Dict[int, Set[int]] = {}
        pending: List[int] = [blocker]

        def corridor(box: int, cells: Sequence[Cell]) -> CellMask:
            mask = CellMask(board, box)
            for cell in cells:
                mask.block_box(board, box, target, cell)
            return mask

        def require(box: int, region: CellMask):
            """Note that `box` must vacate `region`, and put it at the front.

            Front matters: a box already queued further back never gets
            reprioritised otherwise, and whatever is waiting on it re-derives
            the same pin forever — a livelock that looks like a hopeless layout.
            """
            existing = keep_out.get(box)
            if existing is None:
                keep_out[box] = region
            else:
                existing.merge(region)
            if box in pending:
                pending.remove(box)
            pending.insert(0, box)

        keep_out[blocker] = corridor(blocker, remaining)
        short = corridor(blocker, remaining[:self.short_horizon])

        for _ in range(self.max_clears):
            if not pending:
                return legs, state
            box = pending[0]
            if box == target:
                pending.pop(0)
                continue

            solid = CellMask(board, box)
            for other, cell in enumerate(state.cells):
                if other != box:
                    solid.block_box(board, box, other, cell)

            wanted = keep_out[box]
            escape = _bfs(box, state.cells[box], solid,
                          lambda cell: not wanted.blocked(cell))
            if escape is None and box == blocker:
                # Settle for getting out of the immediate way.
                wanted = short
                escape = _bfs(box, state.cells[box], solid,
                              lambda cell: not wanted.blocked(cell))
            if escape is not None:
                pending.pop(0)
                if len(escape) > 1:
                    walked = self._walk(state, box, escape)
                    if walked is None:
                        return None
                    state = walked
                    legs.append((box, escape))
                continue

            # Hemmed in.  Work out the route it *would* have taken with the
            # other boxes gone, and make whatever sits on that route clear it.
            through = _bfs(box, state.cells[box], CellMask(board, box),
                           lambda cell: not wanted.blocked(cell))
            if through is None:
                return None              # walled in by the arena, not a box

            crossers: List[int] = []
            for cell in through:
                for other, at in enumerate(state.cells):
                    if other in (box, target) or other in crossers:
                        continue
                    if board.boxes_overlap(box, cell, other, at):
                        crossers.append(other)

            # Asking the same box again after it failed to help just spins, so
            # work along the route to the next candidate instead.
            already = attempted.setdefault(box, set())
            pin = next((c for c in crossers if c not in already), None)
            if pin is None:
                return None
            already.add(pin)

            region = CellMask(board, pin)
            for cell in through:
                region.block_box(board, pin, box, cell)
            require(pin, region)
        return None

    # --------------------------------------------------------------- assembly

    def _assemble(self, start: PuzzleState, target: int,
                  legs: Sequence[Leg]) -> Optional[Plan]:
        """Replay the legs as single-cell moves, checking every layout."""
        board = self.board
        state = start
        moves: List[Move] = []
        states: List[PuzzleState] = [start]

        for box, path in legs:
            for previous, cell in zip(path, path[1:]):
                if not board.can_place(state, box, cell):
                    return None
                delta = (cell[0] - previous[0], cell[1] - previous[1])
                state = state.with_move(box, delta[0], delta[1])
                moves.append(Move(box, delta, board.step))
                states.append(state)

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

        shoved = len({box for box, _ in legs} - {target})
        return Plan(
            found=True,
            message=f"feasible plan, {shoved} box(es) moved aside",
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

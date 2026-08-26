"""Improve a plan by rewriting it, rather than searching for a different one.

`AnytimePlanner` optimises by *ignoring* the plan it was given except as a price
to beat: it searches the whole joint space again and keeps whatever comes out
cheaper.  On a crowded floor that search gets nowhere, so the plan it hands back
is the one it started with, unchanged.

This works the other way round.  It takes the route decomposition found and edits
it — drop a shove that turned out not to be needed, straighten a detour, roll the
target's stop-start driving into one run — checking after every edit that the plan
still survives being executed.  Each edit is local and cheap, and because they are
verified rather than reasoned about, a wrong guess costs nothing but the attempt.

The edits, cheapest win first:

  drop        Take out everything one box did.  If the plan still works, that
              box never needed moving: saves its travel *and* its grip.
  front-load  Move every shove ahead of the target's first step so the target
              drives in one run.  Decomposition interleaves because it clears
              just-in-time, but the clearing often does not actually depend on
              the target having advanced — and one grip beats seven.
  straighten  Re-plan a single leg between its own endpoints against the floor
              as it stands at that moment.  Clearing happens greedily, so a box
              shoved early often had a shorter way round once things settled.
  collapse    Where a box was shoved twice, send it straight to where it ended
              up.  Saves a grip and usually some distance.

Applied in rounds until nothing improves.  Every candidate is priced with the
same `CostModel` the rest of the study uses, so "better" means the same thing
here as everywhere else.
"""

import time
from typing import Dict, List, Optional, Sequence, Tuple

from astar_planner import Move, Plan
from board import Board
from cost_model import CostModel
from decomposition import CellMask, _bfs
from puzzle_state import Cell, PuzzleState

Leg = Tuple[int, List[Cell]]


def legs_of(plan: Plan) -> List[Leg]:
    """Split a plan back into per-grip legs: one box, the cells it walks."""
    legs: List[Leg] = []
    box: Optional[int] = None
    path: List[Cell] = []
    for index, move in enumerate(plan.moves):
        before = plan.states[index].cells[move.box]
        after = plan.states[index + 1].cells[move.box]
        if move.box != box:
            if box is not None:
                legs.append((box, path))
            box = move.box
            path = [before]
        path.append(after)
    if box is not None:
        legs.append((box, path))
    return legs


def replay(board: Board, cost: CostModel, start: PuzzleState, target: int,
           goal: Cell, legs: Sequence[Leg]) -> Optional[Plan]:
    """Execute legs move by move, refusing anything the rules reject.

    This is the only thing that decides whether an edit was allowed, which is
    what makes the edits safe to attempt blindly.
    """
    state = start
    moves: List[Move] = []
    states: List[PuzzleState] = [start]

    for box, path in legs:
        for previous, cell in zip(path, path[1:]):
            delta = (cell[0] - previous[0], cell[1] - previous[1])
            if delta != (0, 0) and abs(delta[0]) + abs(delta[1]) != 1:
                return None
        walked = board.carry_path(state, box, path)
        if walked is None:
            return None
        for previous, nxt in zip(walked, walked[1:]):
            before = previous.cells[box]
            after = nxt.cells[box]
            moves.append(Move(box, (after[0] - before[0], after[1] - before[1]),
                              board.step))
            states.append(nxt)
        state = walked[-1]

    if state.cells[target] != goal:
        return None

    distance_by_box: Dict[int, float] = {}
    regrips = 0
    total = 0.0
    previous_box = start.last_moved
    for move in moves:
        regrip = move.box != previous_box
        regrips += int(regrip)
        total += cost.move_cost(move.distance, regrip)
        distance_by_box[move.box] = (distance_by_box.get(move.box, 0.0)
                                     + move.distance)
        previous_box = move.box

    return Plan(
        found=True,
        message="repaired plan",
        moves=moves,
        states=states,
        cost=total,
        distance=sum(distance_by_box.values()),
        regrips=regrips,
        distance_by_box=distance_by_box,
        bound=float("inf"),
    )


class PlanRepairer:
    """Rewrites a feasible plan into a cheaper one that still works."""

    # Each round applies at most one edit and then starts over, because an
    # accepted edit changes what the others see.  Rounds are milliseconds, so
    # the cap is generous and the time budget is what normally stops it.
    def __init__(self, board: Board, cost_model: CostModel, rounds: int = 400,
                 max_seconds: float = 2.0):
        self.board = board
        self.cost = cost_model
        self.rounds = rounds
        self.max_seconds = max_seconds

    def refine(self, start: PuzzleState, target: int, goal: Cell,
               plan: Plan) -> Plan:
        """Return the cheapest plan reachable by editing `plan`.

        Never returns anything worse, and never anything illegal — the original
        comes back untouched if no edit survives.
        """
        best = plan
        legs = legs_of(plan)
        deadline = time.perf_counter() + self.max_seconds

        for _ in range(self.rounds):
            if time.perf_counter() > deadline:
                break
            improved = False
            for candidate in (self._without_each_box(legs, target),
                              self._grouped(legs),
                              self._front_loaded(legs, target),
                              self._straightened(start, target, legs),
                              self._collapsed(start, target, legs)):
                for edited in candidate:
                    priced = replay(self.board, self.cost, start, target, goal,
                                    edited)
                    if priced is not None and priced.cost < best.cost - 1e-9:
                        best, legs = priced, legs_of(priced)
                        improved = True
                        break
                if improved:
                    break
            if not improved:
                break

        if best is not plan:
            best.message = (f"repaired: {plan.cost:.3f} -> {best.cost:.3f}")
        return best

    # ------------------------------------------------------------------ edits

    def _without_each_box(self, legs: Sequence[Leg], target: int):
        """Try removing everything each displaced box did."""
        for box in sorted({b for b, _ in legs} - {target}):
            yield [leg for leg in legs if leg[0] != box]

    def _grouped(self, legs: Sequence[Leg]):
        """Slide a leg next to another leg of the same box.

        A grip is just a change of which box is moving, so A,B,A costs three and
        A,A,B costs two — the moves are identical, only the order differs.  This
        is the edit that matters when re-grips are what you are paying for:
        every pair it manages to make adjacent is one grip saved.

        Both directions are tried, because whether a leg can move earlier or the
        other one later depends entirely on what happens in between.
        """
        positions: Dict[int, List[int]] = {}
        for index, (box, _) in enumerate(legs):
            positions.setdefault(box, []).append(index)

        for box, where in positions.items():
            if len(where) < 2:
                continue
            for earlier, later in zip(where, where[1:]):
                if later == earlier + 1:
                    continue            # already adjacent, nothing to save
                rest = list(legs)
                moving = rest.pop(later)
                yield rest[:earlier + 1] + [moving] + rest[earlier + 1:]

                rest = list(legs)
                moving = rest.pop(earlier)
                yield rest[:later - 1] + [moving] + rest[later - 1:]

    def _front_loaded(self, legs: Sequence[Leg], target: int):
        """Every shove first, then the target drives its whole route in one go."""
        shoves = [leg for leg in legs if leg[0] != target]
        drives = [leg for leg in legs if leg[0] == target]
        if len(drives) < 2:
            return
        whole: List[Cell] = [drives[0][1][0]]
        for _, path in drives:
            whole.extend(path[1:])
        yield shoves + [(target, whole)]

    def _straightened(self, start: PuzzleState, target: int,
                      legs: Sequence[Leg]):
        """Re-plan each leg between its own endpoints, in its real context."""
        board = self.board
        for index, (box, path) in enumerate(legs):
            if len(path) < 3:
                continue
            state = self._state_before(start, legs, index)
            if state is None:
                continue
            solid = CellMask(board, box)
            for other, cell in enumerate(state.cells):
                if other != box:
                    solid.block_box(board, box, other, cell)
            goal_cell = path[-1]
            shorter = _bfs(box, path[0], solid, lambda cell: cell == goal_cell)
            if shorter is not None and len(shorter) < len(path):
                yield list(legs[:index]) + [(box, shorter)] + list(legs[index + 1:])

    def _collapsed(self, start: PuzzleState, target: int, legs: Sequence[Leg]):
        """Where a box was shoved more than once, send it straight to the end."""
        board = self.board
        counts: Dict[int, int] = {}
        for box, _ in legs:
            counts[box] = counts.get(box, 0) + 1

        for box, times in counts.items():
            if box == target or times < 2:
                continue
            first = next(i for i, (b, _) in enumerate(legs) if b == box)
            last_path = [path for b, path in legs if b == box][-1]
            state = self._state_before(start, legs, first)
            if state is None:
                continue
            solid = CellMask(board, box)
            for other, cell in enumerate(state.cells):
                if other != box:
                    solid.block_box(board, box, other, cell)
            destination = last_path[-1]
            direct = _bfs(box, state.cells[box], solid,
                          lambda cell: cell == destination)
            if direct is None:
                continue
            edited: List[Leg] = []
            for i, leg in enumerate(legs):
                if i == first:
                    edited.append((box, direct))
                elif leg[0] != box:
                    edited.append(leg)
            yield edited

    # ---------------------------------------------------------------- helpers

    def _state_before(self, start: PuzzleState, legs: Sequence[Leg],
                      index: int) -> Optional[PuzzleState]:
        """The layout just before leg `index` runs."""
        state = start
        for box, path in legs[:index]:
            walked = self.board.carry_path(state, box, path)
            if walked is None:
                return None
            state = walked[-1]
        return state

"""A* over box layouts.

A search node is a whole layout (`PuzzleState`), not a position — moving box A
out of the way is as much a step as moving the target box towards its goal, and
that is the point of the study.  Edges come from `Board.neighbors` (one box, one
grid step) and are priced by `CostModel`.

The state space is enormous, so two things keep it tractable: the heuristic only
looks at the target box (everything else is free to be anywhere), and the caller
supplies a node budget.  A search that blows the budget reports it rather than
running forever, and a `threading.Event` can cancel one mid-flight so a GUI stays
responsive.
"""

import heapq
import itertools
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from board import Board
from cost_model import CostModel
from puzzle_state import Cell, PuzzleState


@dataclass(frozen=True)
class Move:
    """One box sliding one grid step."""

    box: int
    delta: Cell
    distance: float


@dataclass
class Plan:
    """Result of a search — successful or not."""

    found: bool
    message: str
    moves: List[Move] = field(default_factory=list)
    states: List[PuzzleState] = field(default_factory=list)
    cost: float = 0.0
    distance: float = 0.0
    regrips: int = 0
    distance_by_box: Dict[int, float] = field(default_factory=dict)
    nodes_expanded: int = 0
    nodes_generated: int = 0
    elapsed: float = 0.0
    # Suboptimality guarantee: this plan costs at most `bound` times the best
    # possible one.  1.0 means proven optimal.
    bound: float = 1.0

    @property
    def boxes_moved(self) -> List[int]:
        return sorted(self.distance_by_box)


class AStarPlanner:
    """Finds the cheapest sequence of single-step box slides to a goal cell."""

    def __init__(self, board: Board, cost_model: CostModel, max_nodes: int = 120_000):
        self.board = board
        self.cost = cost_model
        self.max_nodes = max_nodes

    def plan(self, start: PuzzleState, target: int, goal: Cell,
             cancel=None) -> Plan:
        """Plan a move of box `target` onto grid cell `goal`."""
        board, cost = self.board, self.cost
        started = time.perf_counter()

        if not board.in_bounds(target, goal):
            return Plan(False, "goal lies outside the arena")

        counter = itertools.count()
        best: Dict[PuzzleState, float] = {start: 0.0}
        came: Dict[PuzzleState, Tuple[PuzzleState, Move]] = {}
        # Heap key: (f, -g, tie-break counter, state).  Preferring the larger g
        # among equal-f nodes drives the search at the goal instead of fanning
        # out across a plateau of equally promising layouts; the counter keeps
        # states themselves out of the comparison.
        open_heap = [(cost.heuristic(board, start, target, goal), -0.0,
                      next(counter), start)]
        closed = set()
        generated = 0

        while open_heap:
            _, _, _, state = heapq.heappop(open_heap)
            if state in closed:
                continue
            closed.add(state)

            if state.cells[target] == goal:
                return self._reconstruct(start, state, came, len(closed), generated,
                                         time.perf_counter() - started)

            if len(closed) >= self.max_nodes:
                return Plan(
                    False,
                    f"node budget of {self.max_nodes:,} reached — raise the budget, "
                    "raise the heuristic weight, or coarsen the grid",
                    nodes_expanded=len(closed), nodes_generated=generated,
                    elapsed=time.perf_counter() - started,
                )

            # Checked occasionally rather than every node: the Event is cheap but
            # the search loop is hot.
            if cancel is not None and len(closed) % 512 == 0 and cancel.is_set():
                return Plan(False, "search cancelled",
                            nodes_expanded=len(closed), nodes_generated=generated,
                            elapsed=time.perf_counter() - started)

            g = best[state]
            for nxt, box, delta, distance in board.neighbors(state):
                if nxt in closed:
                    continue
                generated += 1
                ng = g + cost.move_cost(distance, box != state.last_moved)
                if ng < best.get(nxt, float("inf")) - 1e-12:
                    best[nxt] = ng
                    came[nxt] = (state, Move(box, delta, distance))
                    f = ng + cost.heuristic(board, nxt, target, goal)
                    heapq.heappush(open_heap, (f, -ng, next(counter), nxt))

        return Plan(False, "no legal sequence of moves reaches that goal",
                    nodes_expanded=len(closed), nodes_generated=generated,
                    elapsed=time.perf_counter() - started)

    # ------------------------------------------------------------------------

    def _reconstruct(self, start: PuzzleState, goal_state: PuzzleState,
                     came: Dict[PuzzleState, Tuple[PuzzleState, Move]],
                     expanded: int, generated: int, elapsed: float) -> Plan:
        return reconstruct(self.cost, start, goal_state, came, expanded,
                           generated, elapsed)


def reconstruct(cost: CostModel, start: PuzzleState, goal_state: PuzzleState,
                came: Dict[PuzzleState, Tuple[PuzzleState, Move]],
                expanded: int, generated: int, elapsed: float) -> Plan:
    """Walk the predecessor chain back into a plan, and price it.

    Shared with `anytime_planner` so the two cannot disagree about what a plan
    costs — the cost is always recomputed from the moves themselves rather than
    carried along from the search.
    """
    moves: List[Move] = []
    states: List[PuzzleState] = [goal_state]
    node = goal_state
    while node in came:
        node, move = came[node]
        moves.append(move)
        states.append(node)
    moves.reverse()
    states.reverse()

    distance_by_box: Dict[int, float] = {}
    regrips = 0
    previous = start.last_moved
    total_cost = 0.0
    for move in moves:
        regrip = move.box != previous
        regrips += int(regrip)
        total_cost += cost.move_cost(move.distance, regrip)
        distance_by_box[move.box] = distance_by_box.get(move.box, 0.0) + move.distance
        previous = move.box

    return Plan(
        found=True,
        message="plan found" if moves else "box is already on the goal",
        moves=moves,
        states=states,
        cost=total_cost,
        distance=sum(distance_by_box.values()),
        regrips=regrips,
        distance_by_box=distance_by_box,
        nodes_expanded=expanded,
        nodes_generated=generated,
        elapsed=elapsed,
    )

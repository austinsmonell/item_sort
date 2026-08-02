"""Anytime planning: a feasible plan straight away, then keep improving it.

Plain A* is all-or-nothing.  On a crowded floor it either returns the optimal
plan or, after half a minute, nothing at all — and you cannot tell which it will
be until it is over.  That is the wrong shape for something a person is sitting
in front of.

This runs the same A*, repeatedly, with the heuristic inflated by a weight that
starts high and comes down:

    weight 5.0   greedy.  Drives at the goal, ignores most alternatives, and
                 usually has *a* legal plan in milliseconds.  It may be up to
                 5x more expensive than the best possible one.
    weight 2.5   less greedy, better plan, more search.
    ...
    weight 1.0   plain admissible A*.  Optimal, and slow when the floor is full.

Every time a run finds a plan cheaper than the one in hand, it is handed to the
caller immediately through `on_improve`, along with the guarantee that it is
within `weight` times optimal.  So the plan on screen only ever gets better, the
bound printed beside it only ever gets tighter, and stopping early is always
safe: whatever is in hand is legal and its bound is known.

Two things stop the later, expensive rounds wasting effort:

* **Incumbent pruning.**  Once a plan costing C is in hand, any node whose
  `g + h` already reaches C cannot lead anywhere better, because `h` never
  overestimates.  Those are dropped on sight.  Later rounds are therefore much
  cheaper than running them cold.
* **A shared node budget.**  Rounds spend from one pool, so a hopeless final
  round cannot run forever.

Restarting each round rather than repairing the previous search tree (what ARA*
does) throws away work.  It also keeps this to one readable loop, and with
incumbent pruning the repeated rounds are cheap enough that the trade reads well
for a study.
"""

import heapq
import itertools
import time
from typing import Dict, List, Optional, Sequence, Tuple

from astar_planner import Move, Plan, reconstruct
from board import Board
from cost_model import CostModel
from puzzle_state import Cell, PuzzleState

# Inflation schedule, greedy first.  Anything at or below the weight the user
# asked for is dropped, and their weight is always the last round — so with the
# slider at 1.0 the search ends on a proven optimum.
#
# It opens at 25 rather than something modest because the first round has one
# job: return *a* plan.  Measured on a 24-box floor at 42% fill, reaching a far
# corner took 6.8 s at weight 5 and 2.0 s at weight 25 — worse plans, far
# sooner, and the later rounds are there to fix the cost.
DEFAULT_WEIGHTS: Tuple[float, ...] = (25.0, 10.0, 5.0, 3.0, 2.0, 1.5, 1.25)


class AnytimePlanner:
    """Weighted A* run repeatedly, tightening towards optimal."""

    def __init__(self, board: Board, cost_model: CostModel,
                 max_nodes: int = 120_000,
                 weights: Sequence[float] = DEFAULT_WEIGHTS,
                 max_seconds: Optional[float] = None):
        self.board = board
        self.cost = cost_model
        self.max_nodes = max_nodes
        self.weights = weights
        self.max_seconds = max_seconds

    def plan(self, start: PuzzleState, target: int, goal: Cell,
             cancel=None, on_improve=None, seed: Optional[Plan] = None) -> Plan:
        """Improve a plan until it is optimal, the budget runs out, or `cancel`.

        `on_improve(plan)` is called for every plan better than the last, with
        `plan.bound` set to what the guarantee is worth at that point.  The
        return value is the best plan found, or a failure if there was none.

        `seed` is a plan to start from — normally the one `decomposition` found
        in a millisecond.  It is not re-announced through `on_improve` (whoever
        supplied it has already seen it), but it prices every round from the
        first: with a plan costing C in hand, whole branches are cut on sight.
        """
        board = self.board
        started = time.perf_counter()
        if not board.in_bounds(target, goal):
            return Plan(False, "goal lies outside the arena")

        final_weight = max(self.cost.heuristic_weight, 1.0)
        schedule = [w for w in self.weights if w > final_weight]
        schedule.append(final_weight)

        best: Optional[Plan] = seed if seed is not None and seed.found else None
        expanded = generated = 0
        exhausted = False
        stopped = ""

        for weight in schedule:
            budget = self.max_nodes - expanded
            if budget <= 0:
                stopped = f"node budget of {self.max_nodes:,} used up"
                break
            if self._out_of_time(started):
                stopped = f"time limit of {self.max_seconds:g} s reached"
                break
            if cancel is not None and cancel.is_set():
                stopped = "stopped by request"
                break

            found, complete, spent, seen = self._search(
                start, target, goal, weight,
                None if best is None else best.cost, budget, cancel, started)
            expanded += spent
            generated += seen

            if found is not None and (best is None
                                      or found.cost < best.cost - 1e-9):
                found.bound = weight
                found.nodes_expanded = expanded
                found.nodes_generated = generated
                found.elapsed = time.perf_counter() - started
                best = found
                if on_improve is not None:
                    on_improve(best)

            # Only the final round's completion says anything about the
            # incumbent.  Earlier rounds order by an inflated `f` and skip
            # already-closed states, so emptying their open list does not rule
            # out a cheaper plan; the guarantee for those comes from the weight
            # of the round that *found* the plan, which is set above.
            if complete and best is not None and weight <= final_weight + 1e-9:
                best.bound = min(best.bound, final_weight)
                exhausted = True
                break
            if complete and best is None and weight <= final_weight + 1e-9:
                # Searched the whole space and there is genuinely no plan.
                return Plan(False, "no legal sequence of moves reaches that goal",
                            nodes_expanded=expanded, nodes_generated=generated,
                            elapsed=time.perf_counter() - started)

        if best is None:
            return Plan(
                False,
                (stopped or f"node budget of {self.max_nodes:,} reached")
                + " before any plan was found — raise the budget, coarsen the "
                  "grid, or pick a nearer goal",
                nodes_expanded=expanded, nodes_generated=generated,
                elapsed=time.perf_counter() - started)

        best.nodes_expanded = expanded
        best.nodes_generated = generated
        best.elapsed = time.perf_counter() - started
        if exhausted and best.bound <= final_weight + 1e-9:
            best.message = ("optimal plan" if final_weight <= 1.0 + 1e-9
                            else f"best within {final_weight:g}x optimal")
        elif best.bound == float("inf"):
            best.message = "feasible plan (no optimality bound yet)"
        elif stopped:
            best.message = f"best plan so far ({stopped})"
        else:
            best.message = f"best plan so far (within {best.bound:g}x optimal)"
        return best

    # ------------------------------------------------------------------------

    def _out_of_time(self, started: float) -> bool:
        return (self.max_seconds is not None
                and time.perf_counter() - started > self.max_seconds)

    def _search(self, start: PuzzleState, target: int, goal: Cell,
                weight: float, incumbent: Optional[float], budget: int,
                cancel, started: float):
        """One weighted A* round.

        Returns (plan or None, ran_to_completion, nodes expanded, edges seen).
        `ran_to_completion` means the round emptied its open list rather than
        being cut short — only then does its weight say anything about how good
        the incumbent is.
        """
        board, cost = self.board, self.cost
        counter = itertools.count()
        best_g: Dict[PuzzleState, float] = {start: 0.0}
        came: Dict[PuzzleState, Tuple[PuzzleState, Move]] = {}
        closed = set()
        generated = 0

        root_h = cost.heuristic(board, start, target, goal, weight=1.0)
        open_heap = [(weight * root_h, -0.0, next(counter), start)]

        while open_heap:
            _, _, _, state = heapq.heappop(open_heap)
            if state in closed:
                continue
            closed.add(state)

            if state.cells[target] == goal:
                plan = reconstruct(cost, start, state, came, len(closed),
                                   generated, time.perf_counter() - started)
                return plan, False, len(closed), generated

            if len(closed) >= budget:
                return None, False, len(closed), generated
            if len(closed) % 512 == 0:
                if cancel is not None and cancel.is_set():
                    return None, False, len(closed), generated
                if self._out_of_time(started):
                    return None, False, len(closed), generated

            g = best_g[state]
            for nxt, box, delta, distance in board.neighbors(state):
                if nxt in closed:
                    continue
                generated += 1
                ng = g + cost.move_cost(distance, box != state.last_moved)
                if ng >= best_g.get(nxt, float("inf")) - 1e-12:
                    continue
                h = cost.heuristic(board, nxt, target, goal, weight=1.0)
                # `h` never overestimates, so g + h is a floor on what any plan
                # through here can cost.  If that already matches the plan in
                # hand, this branch cannot improve on it.
                if incumbent is not None and ng + h >= incumbent - 1e-12:
                    continue
                best_g[nxt] = ng
                came[nxt] = (state, Move(box, delta, distance))
                heapq.heappush(open_heap,
                               (ng + weight * h, -ng, next(counter), nxt))

        return None, True, len(closed), generated

"""The tunable cost function the search minimises.

Two terms, both asked for by the study:

  distance_weight  cost per metre travelled, by *any* box.  This is the "total
                   distance of all moved boxes" term.
  regrip_weight    cost charged every time the mechanism has to start moving a
                   different box than the one it moved last.  This is the
                   "number of boxes required to move" term: a plan that shoves
                   three boxes aside pays it three extra times, and a box that
                   is pushed, released, then pushed again pays it twice.

`heuristic_weight` is not part of the cost — it inflates the heuristic instead.
At 1.0 A* returns a provably optimal plan.  Above 1.0 the search is greedier: it
expands far fewer nodes and returns a plan whose cost is at most that factor
above optimal.  It is the knob to reach for when a plan is taking too long.
"""


class CostModel:
    """Weights for a plan's cost, plus the matching A* heuristic."""

    def __init__(self, distance_weight: float = 1.0,
                 regrip_weight: float = 0.25,
                 heuristic_weight: float = 1.0):
        self.distance_weight = distance_weight
        self.regrip_weight = regrip_weight
        self.heuristic_weight = heuristic_weight

    def move_cost(self, distance: float, regrip: bool) -> float:
        """Cost of sliding one box `distance` metres."""
        cost = self.distance_weight * distance
        if regrip:
            cost += self.regrip_weight
        return cost

    def heuristic(self, board, state, target: int, goal) -> float:
        """Optimistic cost from `state` to having `target` sit on `goal`.

        The target box must itself travel at least the Manhattan distance to the
        goal (moves are four-connected and cost the same per metre whichever box
        makes them), and if it is not already the box in hand that travel needs
        at least one more grip.  Both terms understate the truth, so at
        heuristic_weight = 1.0 the search stays admissible.
        """
        cx, cy = state.cells[target]
        steps = abs(goal[0] - cx) + abs(goal[1] - cy)
        if steps == 0:
            return 0.0

        estimate = self.distance_weight * steps * board.step
        if state.last_moved != target:
            estimate += self.regrip_weight
        return self.heuristic_weight * estimate

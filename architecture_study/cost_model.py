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

    def heuristic(self, board, state, target: int, goal,
                  weight: float = None) -> float:
        """Optimistic cost from `state` to having `target` sit on `goal`.

        Two sources of guaranteed remaining cost:

        * The target must itself travel at least the Manhattan distance to the
          goal (moves are four-connected and cost the same per metre whichever
          box makes them), and unless it is already the box in hand that travel
          needs a grip.

        * Any box sitting on the goal footprint *provably* has to move, since
          the target cannot end up overlapping it.  Each one needs at least
          enough travel to stop overlapping, plus a grip of its own unless it is
          the box already in hand.

        Every term belongs to a different box and to a different grip, so they
        add without ever overshooting the true remaining cost — which is what
        keeps A* optimal at heuristic_weight = 1.0.  Ignoring boxes that merely
        sit *between* the target and its goal is deliberate: the target may be
        able to go around them, so charging for them would not be admissible.
        """
        cells = state.cells
        cx, cy = cells[target]
        steps = abs(goal[0] - cx) + abs(goal[1] - cy)
        if steps == 0:
            return 0.0

        held = state.last_moved
        estimate = self.distance_weight * steps * board.step
        if held != target:
            estimate += self.regrip_weight

        goal_rect = None
        for j, cell in enumerate(cells):
            if j == target or not board.boxes_overlap(target, goal, j, cell):
                continue
            if goal_rect is None:
                goal_rect = board.rect(target, goal)
            other = board.rect(j, cell)
            # Shallowest way out of the goal footprint, in any of the four
            # directions — a lower bound on how far this box must travel.
            clearance = min(goal_rect.x2 - other.x, other.x2 - goal_rect.x,
                            goal_rect.y2 - other.y, other.y2 - goal_rect.y)
            estimate += self.distance_weight * clearance
            if j != held:
                estimate += self.regrip_weight

        # `weight` overrides the model's own inflation for one call, which is how
        # the anytime planner gets both the raw admissible estimate (for its
        # optimality bound) and an inflated one (for search order) without
        # disturbing the shared model the GUI is editing.
        return estimate * (self.heuristic_weight if weight is None else weight)

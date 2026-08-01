"""Shared constants for the study — arena, boxes, grid and default weights.

Every module reads its defaults from here so the planner and the GUI cannot
disagree.  Lengths are metres throughout.
"""

from geometry import Rect

# --------------------------------------------------------------------- layout

ARENA = Rect(0.0, 0.0, 1.20, 0.80)

# (name, x, y, w, h) — x, y is the lower-left corner.  Corners are snapped to the
# grid at load time, so a layout that only fits on a fine grid will be rejected
# when the grid is coarsened.
BOXES = [
    ("A", 0.08, 0.08, 0.20, 0.16),
    ("B", 0.40, 0.08, 0.32, 0.12),
    ("C", 0.84, 0.08, 0.16, 0.28),
    ("D", 0.12, 0.44, 0.24, 0.20),
    ("E", 0.52, 0.36, 0.12, 0.12),
    ("F", 0.76, 0.52, 0.28, 0.16),
]

# Box corners may only sit on multiples of this.  It is the single biggest lever
# on search time: halving it quadruples the reachable layouts.
GRID_STEP = 0.04
GRID_STEP_RANGE = (0.02, 0.10)

# ---------------------------------------------------------------- cost weights

DISTANCE_WEIGHT = 1.0    # cost per metre travelled by any box
REGRIP_WEIGHT = 0.25     # cost each time the mechanism switches box
HEURISTIC_WEIGHT = 1.0   # 1.0 = optimal plan; higher = greedier and faster

DISTANCE_WEIGHT_RANGE = (0.0, 5.0)
REGRIP_WEIGHT_RANGE = (0.0, 2.0)
HEURISTIC_WEIGHT_RANGE = (1.0, 3.0)

# --------------------------------------------------------------------- search

MAX_NODES = 120_000
MAX_NODES_RANGE = (20_000, 400_000)

# -------------------------------------------------------------------- display

WINDOW_TITLE = "A* rectangle sorting study"
CANVAS_SIZE = (760, 520)
PLAYBACK_MS = 40          # milliseconds per animated grid step
PLAYBACK_MS_RANGE = (5, 200)

BOX_COLORS = [
    "#4C78A8", "#F58518", "#54A24B", "#E45756",
    "#72B7B2", "#B279A2", "#EECA3B", "#9D755D",
]

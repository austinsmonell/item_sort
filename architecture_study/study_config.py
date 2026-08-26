"""Shared constants for the study — arena, box catalogue, grid and weights.

Every module reads its defaults from here so the planner and the GUI cannot
disagree.  Lengths are metres throughout.
"""

from geometry import Rect

# --------------------------------------------------------------------- layout

ARENA = Rect(0.0, 0.0, 1.8, 1.2)

# The standard box catalogue: (type name, side length).  Boxes are squares, and
# a layout is described by how many of each type you want rather than by
# hand-written rectangles — see layout.LayoutGenerator.
BOX_TYPES = (
    ("small", 0.10),
    ("medium", 0.20),
    ("large", 0.30),
)

DEFAULT_COUNTS = {"small": 4, "medium": 3, "large": 2}

# The gripper: two L-shaped jaws that close on opposite corners of a box.  The
# jaws sit outside the box, so a box needs empty floor at its corners before it
# can be picked up at all — layouts are generated with that clearance in mind.
GRIPPER_THICKNESS = 0.01   # how thick each arm of the L is
GRIPPER_REACH = 0.05       # how far each arm runs along the edge from the corner
GRIPPER_STROKE = 0.02      # how far the travelling jaw backs off to open
MAX_COUNT_PER_TYPE = 40

# Random placement tries this many cells per box before giving up and reporting
# that the layout will not fit.
PLACEMENT_ATTEMPTS = 500
RANDOM_SEED = None  # pin to an integer for a repeatable starting layout

# Box corners may only sit on multiples of this.  It is the single biggest lever
# on search time: halving it quadruples the reachable layouts.
GRID_STEP = 0.05
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
CANVAS_SIZE = (820, 560)
PLAYBACK_MS = 40          # milliseconds per animated grid step
PLAYBACK_MS_RANGE = (5, 200)

# Boxes are coloured by type, so the three standard sizes stay readable however
# many of them are on the floor.
BOX_TYPE_COLORS = {
    "small": "#54A24B",
    "medium": "#4C78A8",
    "large": "#E45756",
}
FALLBACK_BOX_COLOR = "#B279A2"

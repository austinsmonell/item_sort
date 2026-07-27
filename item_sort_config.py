"""Single source of truth for the item_sort gantry.

Geometry, loop rates and control gains live here so the simulator, the
controller and (eventually) the hardware runner behave identically. Change
control params here, not in copies.

Units at the Python boundary are plain SI and *linear*:

    position  m        velocity  m/s        force  N

Both axes are direct linear actuators, so unlike balance_bot / biped there are
no revolutions or torques anywhere -- an axis command is a force in Newtons and
an axis measurement is a position in metres.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Axes
# ---------------------------------------------------------------------------
# Order is fixed by the <actuator> list in item_sort.xml. Everything that
# indexes an axis (gains, forces, travel limits, the [x, y] state vector) uses
# this order.
AXIS_NAMES = ("axis_x", "axis_y")
N_AXES = len(AXIS_NAMES)
X, Y = 0, 1

# Half-travel of each axis, m. Must match the joint `range` in item_sort.xml --
# ItemSortSim checks this at load time.
TRAVEL = np.array([0.57, 0.375])

# Commanded targets stay this far off the hard end stops, so a little tracking
# overshoot cannot jam the axis against a limit (where the PID's integral winds
# up and the paddle never reaches its setpoint).
END_STOP_MARGIN = 0.008
SOFT_TRAVEL = TRAVEL - END_STOP_MARGIN

# What the actuator can physically deliver. Must match the actuator ctrlrange in
# item_sort.xml: ask for more and MuJoCo silently clips it, so anything above
# this is a lie to the controller.
MAX_FORCE = np.array([10.0, 10.0])   # N

# The limit the PID actually clamps its output to, per axis. Starts at the
# actuator ceiling; the GUI exposes it as a slider (0 .. MAX_FORCE) so an axis
# can be run soft -- useful for nudging a box instead of punting it, and for
# seeing which of a PID's problems are really saturation.
FORCE_LIMIT = MAX_FORCE.copy()   # N


# ---------------------------------------------------------------------------
# Loop rates
# ---------------------------------------------------------------------------
# One control loop: the position PID. SIM_HZ is a multiple of CTRL_HZ so every
# control tick lands on an exact physics-step boundary.
SIM_HZ = 100      # physics; also written into model.opt.timestep at startup
CTRL_HZ = 100     # per-axis position PID

assert SIM_HZ % CTRL_HZ == 0, "SIM_HZ must be a multiple of CTRL_HZ"

SIM_DT = 1.0 / SIM_HZ
CTRL_DT = 1.0 / CTRL_HZ
SIM_PER_CTRL = SIM_HZ // CTRL_HZ


# ---------------------------------------------------------------------------
# PID gains (per axis, [x, y])
# ---------------------------------------------------------------------------
# The x axis carries the whole bridge (~3.4 kg moving mass incl. armature), the
# y axis only the small carriage (~0.9 kg), hence the split gains. Roughly
# critically damped around 3 Hz; these are the primary tuning knobs, and the GUI
# exposes them as sliders so they can be trimmed while the sim runs.
KP = np.array([900.0, 320.0])   # N/m
KD = np.array([110.0, 40.0])    # N/(m/s)
KI = np.array([250.0, 90.0])    # N/(m*s)
INTEGRAL_LIMIT = 0.05           # m*s, clamp on the integral state

# Slider ranges for the gain tuners in the GUI.
KP_MAX, KD_MAX, KI_MAX = 2000.0, 400.0, 800.0


# ---------------------------------------------------------------------------
# Platform geometry (mirrors item_sort.xml, work surface top at z = 0)
# ---------------------------------------------------------------------------
PLATFORM_HALF = np.array([0.65, 0.45])   # half-extents of the work surface, m
PADDLE_HALF = 0.045   # half-width of the pusher paddle, m
BOX_HALF = 0.020      # half-width of a box, m

# Boxes are props: there is nothing sorting them, they are just there to shove
# around by hand. Names must match the <body> names in item_sort.xml.
BOX_NAMES = ("box1", "box2", "box3")


def clamp_to_travel(xy):
    """Clamp an (x, y) paddle target into the usable axis travel (soft limits)."""
    return np.clip(np.asarray(xy, dtype=float), -SOFT_TRAVEL, SOFT_TRAVEL)

"""Single source of truth for the item_sort gantry.

Geometry, loop rates and control gains live here so the simulator, the
controller and (eventually) the hardware runner behave identically. Change
control params here, not in copies.

Units at the Python boundary are plain SI, but they are **per axis**: the gantry
has two linear axes and one rotary one, so the axis vector is mixed.

    axis        state     rate       command
    x, y        m         m/s        N
    yaw         rad       rad/s      N*m

AXIS_UNITS / EFFORT_UNITS spell that out for anything that has to print it. The
command vector is still called "force" throughout (MAX_FORCE, set_force,
max_force) because that is what MuJoCo calls every actuator output regardless of
joint type -- on the yaw axis, read those entries as torque in N*m. The joint
side stays in plain radians: unlike ../balance_bot and ../biped, which follow the
moteus convention (rev, rev/s, Nm), nothing here commands revolutions. Revs
appear in one place only, the motor-side rpm readout in the Drivetrain section,
because rpm is how a motor's datasheet is written.

The machine is specified in imperial (a 4 ft platform, 3/6/12 in boxes, a 1/4 in
blade) because that is how the real thing would be built and how the stock
arrives; INCH converts once, here, and nothing downstream sees an inch again.
"""

import numpy as np


INCH = 0.0254   # the only imperial->SI conversion in the project


# ---------------------------------------------------------------------------
# Axes
# ---------------------------------------------------------------------------
# Order is fixed by the <actuator> list in item_sort.xml. Everything that
# indexes an axis (gains, forces, travel limits, the [x, y, yaw] state vector)
# uses this order.
AXIS_NAMES = ("axis_x", "axis_y", "axis_yaw")
AXIS_LABELS = ("X", "Y", "Yaw")
AXIS_UNITS = ("m", "m", "rad")        # of a position / setpoint
EFFORT_UNITS = ("N", "N", "N*m")      # of a command
N_AXES = len(AXIS_NAMES)
X, Y, YAW = 0, 1, 2

# Half-travel of each axis (m, m, rad). Must match the joint `range` in
# item_sort.xml -- ItemSortSim checks this at load time.
#
# x and y are equal because the platform is square, and both are set for the
# WIDEST blade the width slider allows, not the current one: 0.6096 (platform
# half) - 0.1524 (widest blade half) leaves 0.4572, so 0.45 keeps the blade
# clear of the perimeter lip at every width and every yaw angle. Sizing travel
# to the current width instead would mean re-deriving the soft limits every time
# the slider moved, and a real machine's limits are set for its widest tool too.
#
# Yaw runs to +/-100 deg rather than +/-90 so that a square-on push along either
# axis, in either direction, sits inside the travel with the end-stop margin
# still applied -- at exactly +/-90 the useful extreme would be unreachable.
TRAVEL = np.array([0.45, 0.45, np.deg2rad(100.0)])

# Commanded targets stay this far off the hard end stops, so a little tracking
# overshoot cannot jam an axis against a limit (where the PID's integral winds up
# and the axis never reaches its setpoint). Per axis, so the rotary axis gets a
# margin in radians rather than a metre-sized one.
END_STOP_MARGIN = np.array([0.008, 0.008, np.deg2rad(2.0)])
SOFT_TRAVEL = TRAVEL - END_STOP_MARGIN

# What each actuator can physically deliver (N, N, N*m). Must match the actuator
# ctrlrange in item_sort.xml: ask for more and MuJoCo silently clips it, so
# anything above this is a lie to the controller.
MAX_FORCE = np.array([120.0, 80.0, 20.0])

# ---------------------------------------------------------------------------
# Drivetrain: what actually produces the effort on each axis
# ---------------------------------------------------------------------------
# Every axis is a motor rigidly linked to what it moves -- x and y through a
# pinion of pitch radius r driving the carriage, yaw through a gear pair of
# reduction ratio N. **None of this is in item_sort.xml.** MuJoCo still sees a
# plain force/torque actuator on each joint; the drivetrain lives here as the
# conversion between the motor's world (shaft torque, rpm -- how a motor is
# specified and bought) and the joint's (N or N*m, m/s or rad/s -- what the PID
# and the physics deal in):
#
#   linear axis (x, y)   F = tau / r      omega = v / r
#   rotary axis (yaw)    T = tau * N      omega = w * N
#
# Both directions are the same single number per axis, `gear_gain` below: axis
# effort per N*m of motor torque, and motor rad/s per unit of axis speed. Adding
# gear geometry to the model would buy nothing the controller can see, and would
# cost a stiff constraint in the solver.
#
# The default r puts the default motor torque exactly at MAX_FORCE, so out of
# the box the axes drive as hard as the actuators allow.
GEAR_IS_RADIUS = np.array([True, True, False])   # yaw's gear is a plain ratio

MOTOR_TORQUE = np.array([2.4, 1.6, 20.0])        # N*m at the shaft, per axis
MOTOR_TORQUE_MAX = np.array([6.0, 4.0, 50.0])    # slider ceilings

GEAR = np.array([0.020, 0.020, 1.0])             # m on x/y, ratio on yaw
GEAR_MIN = np.array([0.005, 0.005, 0.2])
GEAR_MAX = np.array([0.060, 0.060, 5.0])

# How the GUI writes the gear number: radii read in mm, the ratio as itself.
GEAR_LABELS = ("gear radius", "gear radius", "gear ratio")
GEAR_UNITS = ("mm", "mm", ":1")
GEAR_SCALE = np.array([1000.0, 1000.0, 1.0])     # SI -> slider units

MOTOR_UNITS = ("N*m",) * N_AXES                  # motor-side torque, every axis
RPM_UNITS = ("rpm",) * N_AXES
RAD_S_TO_RPM = 60.0 / (2.0 * np.pi)


def gear_gain(gear):
    """Axis effort per N*m of motor torque -- which is also motor rad/s per unit
    of axis speed, since a rigid link cannot trade one without the other.

    `gear` is the per-axis gear number in SI (pitch radius in m on x and y, a
    dimensionless ratio on yaw). It is clamped into its usable range first, so a
    radius can never reach zero and hand an axis infinite force.
    """
    gear = np.clip(np.asarray(gear, dtype=float), GEAR_MIN, GEAR_MAX)
    return np.where(GEAR_IS_RADIUS, 1.0 / gear, gear)


def axis_effort_limit(torque, gear):
    """What the drivetrain can put on each axis (N, N, N*m), capped by what the
    actuator itself can deliver -- gearing for more force than MAX_FORCE just
    gets clipped by MuJoCo, silently, so cap it here where it is visible."""
    return np.minimum(np.asarray(torque, dtype=float) * gear_gain(gear), MAX_FORCE)


def motor_torque(effort, gear):
    """Shaft torque (N*m) behind a given axis effort (N, N, N*m)."""
    return np.asarray(effort, dtype=float) / gear_gain(gear)


def motor_rpm(vel, gear):
    """Shaft speed (rev/min) at a given axis speed (m/s, m/s, rad/s)."""
    return np.asarray(vel, dtype=float) * gear_gain(gear) * RAD_S_TO_RPM


# The limit the PID actually clamps its output to, per axis: whatever the motor
# and its gearing can deliver, never more than the actuator ceiling. The GUI
# drives this through the torque and gear sliders rather than setting it
# directly -- an axis is run soft by fitting a smaller motor or regearing it, not
# by wishing the force away.
FORCE_LIMIT = axis_effort_limit(MOTOR_TORQUE, GEAR)


# Dry (Coulomb) friction in each axis (N, N, N*m): the effort the drive has to
# overcome before the axis moves at all, independent of speed. This is MuJoCo's
# joint `frictionloss`, and it is what a real belt/leadscrew/rotary stage, its
# bearings and its wipers actually cost you -- the thing that turns a clean PID
# into one that creeps, sticks short of target and hunts. The GUI exposes it per
# axis. Viscous drag (proportional to speed) is the joint `damping` in the XML.
AXIS_FRICTION = np.array([1.5, 0.8, 0.05])
AXIS_FRICTION_MAX = np.array([20.0, 20.0, 2.0])   # slider ceilings


# ---------------------------------------------------------------------------
# Loop rates
# ---------------------------------------------------------------------------
# One control loop: the position PID. SIM_HZ is a multiple of CTRL_HZ so every
# control tick lands on an exact physics-step boundary. Physics runs well above
# the loop rate because contact is the stiff part of this model, and the blade is
# the reason it is this high: at 1/4 in thick, a step long enough to move it most
# of its own thickness can put it through the side of a box.
SIM_HZ = 1000     # physics; also written into model.opt.timestep at startup
CTRL_HZ = 100     # per-axis position PID

assert SIM_HZ % CTRL_HZ == 0, "SIM_HZ must be a multiple of CTRL_HZ"

SIM_DT = 1.0 / SIM_HZ
CTRL_DT = 1.0 / CTRL_HZ
SIM_PER_CTRL = SIM_HZ // CTRL_HZ


# ---------------------------------------------------------------------------
# PID gains (per axis, [x, y, yaw])
# ---------------------------------------------------------------------------
# Gains are in each axis's own units: N/m on x and y, N*m/rad on yaw.
#
# The x axis carries the whole bridge (~5.3 kg moving mass incl. armature), y
# only the carriage and blade (~1.7 kg), and yaw only the blade, whose own
# inertia is small enough that the drive's reflected inertia (the joint
# `armature`, ~0.05 kg*m^2) dominates it. Hence the very different scales.
# Roughly critically damped -- 2 Hz on the linear axes, 4 Hz on yaw -- with the
# joint damping already subtracted out of Kd. These are the primary tuning
# knobs, and the GUI exposes them as sliders so they can be trimmed live.
KP = np.array([760.0, 250.0, 35.0])
KD = np.array([115.0, 35.0, 2.1])
KI = np.array([220.0, 80.0, 12.0])

# Clamp on the integral state, per axis (m*s, m*s, rad*s). Sized so the integral
# can still out-push axis friction at its slider maximum -- clamp it lower and a
# sticky axis parks short of its setpoint forever.
INTEGRAL_LIMIT = np.array([0.15, 0.15, 0.15])

# Slider ranges for the gain tuners in the GUI. Per axis, since yaw's gains are
# in N*m/rad and live two orders of magnitude below the linear axes'.
KP_MAX = np.array([2000.0, 800.0, 200.0])
KD_MAX = np.array([400.0, 150.0, 20.0])
KI_MAX = np.array([600.0, 300.0, 80.0])


# ---------------------------------------------------------------------------
# Platform geometry (mirrors item_sort.xml, work surface top at z = 0)
# ---------------------------------------------------------------------------
PLATFORM_SIDE = 4 * 12 * INCH            # 4 ft square work surface, m
PLATFORM_HALF = np.array([PLATFORM_SIDE / 2, PLATFORM_SIDE / 2])   # 0.6096 m

# Top of the ground plane in item_sort.xml; the platform slab sits on it. Only
# used to work out how high a parked box rides.
GROUND_Z = -0.05


# ---------------------------------------------------------------------------
# Paddle (the blade on the end of the yaw axis)
# ---------------------------------------------------------------------------
# A thin plate, not a block: 1/4 in of stock standing on edge. It hangs from the
# carriage through the yaw hinge, spanning PADDLE_BOTTOM_Z .. PADDLE_TOP_Z so it
# catches a box low on its face (boxes slide instead of tipping) and still
# reaches above the top of one.
#
# Thickness is along the blade's local x and width along its local y, so at yaw
# = 0 the blade faces along x and pushes in +/-x; at yaw = 90 deg it pushes in
# +/-y. That is the point of the rotary axis: one blade, any push direction.
PADDLE_THICKNESS = 0.25 * INCH     # 6.35 mm
PADDLE_BOTTOM_Z = 0.004            # m, just clear of the work surface
PADDLE_TOP_Z = 0.147               # m, above the top of a box (see BOX_HEIGHT)
PADDLE_HEIGHT = PADDLE_TOP_Z - PADDLE_BOTTOM_Z

# Width is a live GUI slider: a narrow blade pokes one box out of a row, a wide
# one sweeps several at once. The ceiling matches the largest box's footprint --
# past that the blade would be wider than anything it has to move, and TRAVEL is
# already sized against it.
PADDLE_WIDTH = 8.0 * INCH          # 0.2032 m, the default
PADDLE_WIDTH_MIN = 2.0 * INCH      # 0.0508 m
PADDLE_WIDTH_MAX = 12.0 * INCH     # 0.3048 m

# Blade mass scales with width, since a wider blade is simply more of the same
# stock. This is deliberate: widening the blade adds moving mass to y and
# inertia to yaw, so the width slider detunes those axes a little and you can
# see it happen. 2.46 kg/m puts the default 8 in blade at ~0.5 kg.
PADDLE_MASS_PER_M = 2.46


def paddle_mass(width):
    """Blade mass (kg) at a given width (m)."""
    return PADDLE_MASS_PER_M * float(width)


# ---------------------------------------------------------------------------
# Boxes
# ---------------------------------------------------------------------------
# Every box is the same height and differs only in footprint -- 3, 6 or 12 in
# square. That is what a real infeed looks like: one carton height off the line,
# several floor sizes. It also means the blade only has to be tall enough for
# one box height, which is what sets PADDLE_TOP_Z and the bridge ride height.
BOX_HEIGHT = 5.0 * INCH                  # 0.127 m
BOX_HALF_HEIGHT = BOX_HEIGHT / 2.0       # 0.0635 m

# Boxes are props: there is nothing sorting them, they are just there to shove
# around by hand. MuJoCo cannot add bodies to a live model, so the XML declares
# a fixed POOL of each size and the GUI "inserts" one by teleporting a spare
# from an off-platform staging row onto the surface. Nothing is on the platform
# until you put it there.
#
# Slot counts are lopsided on purpose: a 4 ft platform holds plenty of 3 in
# boxes and only a handful of 12 in ones.
BOX_KINDS = ("small", "medium", "large")
BOX_SIDE_IN = {"small": 3.0, "medium": 6.0, "large": 12.0}
BOX_SLOTS = {"small": 6, "medium": 4, "large": 3}

BOX_HALF = {k: BOX_SIDE_IN[k] * INCH / 2 for k in BOX_KINDS}   # footprint half, m

# Mass per box size, kg, exposed as a GUI slider per size: what is *in* a carton
# matters more than how big it is. It sets the force needed to break a box loose
# (mu * m * g) and how far it coasts once moving, so it is the other half of the
# friction story -- a heavy small box is a very different push from a light large
# one. Slider ceilings are generous enough to saturate an axis on purpose.
BOX_MASS = {"small": 0.20, "medium": 0.70, "large": 2.50}
BOX_MASS_MAX = {"small": 2.0, "medium": 8.0, "large": 25.0}
BOX_MASS_MIN = 0.02   # kg; a box with no mass at all makes contact degenerate

# A "slot" is an index into BOX_NAMES, i.e. one pre-declared body in the XML.
# Body names must match these, and the order must match the body order in the
# file (ItemSortSim reads each box's size straight out of the model).
BOX_NAMES = tuple(f"{kind}{i + 1}"
                  for kind in BOX_KINDS for i in range(BOX_SLOTS[kind]))
BOX_SLOT_KIND = tuple(kind for kind in BOX_KINDS for _ in range(BOX_SLOTS[kind]))
BOX_KIND_SLOTS = {}
_next = 0
for _kind in BOX_KINDS:
    BOX_KIND_SLOTS[_kind] = tuple(range(_next, _next + BOX_SLOTS[_kind]))
    _next += BOX_SLOTS[_kind]
del _next, _kind

# Sliding friction coefficient between a box and the platform, exposed as a GUI
# slider. This is the number that decides whether the blade shoves a box or just
# skids it, and how far a box coasts after the blade stops -- so it is a knob,
# not a constant. item_sort.xml pins it with explicit contact <pair>s (platform
# <-> each box geom) rather than leaving it to MuJoCo's "max of the two geoms"
# rule, so the value here is the coefficient that actually applies.
SURFACE_FRICTION = 0.5
SURFACE_FRICTION_MIN = 0.02   # near-frictionless: boxes glide and overshoot
SURFACE_FRICTION_MAX = 1.5    # grippy: the axis saturates before a box moves

# Clearance a freshly inserted box keeps from the walls, the blade and every box
# already down, m. Insertion fails rather than dropping a box on top of
# something.
BOX_SPAWN_MARGIN = 0.02

# Off-platform staging row: where the pool waits. Parked boxes rest on the
# ground plane well clear of the platform, so they are out of the way but still
# visible if you zoom out. item_sort.xml gives each body its park pose as its
# initial `pos`, and ItemSortSim re-asserts these on reset.
BOX_PARK_Y = 2.2
BOX_PARK_X0 = -2.7
BOX_PARK_PITCH = 0.45


def box_park_pose(slot):
    """Staging pose (x, y, z) of box `slot` when it is not on the platform."""
    return np.array([BOX_PARK_X0 + slot * BOX_PARK_PITCH,
                     BOX_PARK_Y,
                     GROUND_Z + BOX_HALF_HEIGHT])


def clamp_to_travel(xyz):
    """Clamp an (x, y, yaw) axis target into the usable travel (soft limits)."""
    return np.clip(np.asarray(xyz, dtype=float), -SOFT_TRAVEL, SOFT_TRAVEL)

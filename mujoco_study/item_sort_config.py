"""Single source of truth for the item_sort gantry.

Geometry, loop rates and control gains live here so the simulator, the
controller and (eventually) the hardware runner behave identically. Change
control params here, not in copies.

Units at the Python boundary are plain SI and **linear** -- all three axes are
slides, so every one reads the same:

    axis          state     rate       command
    x, y, jaw     m         m/s        N

AXIS_UNITS / EFFORT_UNITS spell that out for anything that has to print it. The
command vector is called "force" throughout (MAX_FORCE, set_force, max_force),
which is both what MuJoCo calls an actuator output and what these actually are.
Nothing here commands revolutions or torque at the joint: unlike ../balance_bot
and ../biped, which follow the moteus convention (rev, rev/s, Nm), the axes are
linear. Torque and rpm appear in one place only -- the Drivetrain section, on the
*motor* side of the gearing -- because that is how a motor is specified, bought
and sized.

The machine is specified in imperial (a 4 ft platform, 3/6/12 in boxes, 1/4 in
bracket stock) because that is how the real thing would be built and how the
stock arrives; INCH converts once, here, and nothing downstream sees an inch
again.

Sections are ordered by dependency, not by importance: the platform and the tool
come first because the axis travel is derived from how far the tool sticks out.
"""

import numpy as np


INCH = 0.0254   # the only imperial->SI conversion in the project
DIAG = np.sqrt(2.0)


# ---------------------------------------------------------------------------
# Platform geometry (mirrors item_sort.xml, work surface top at z = 0)
# ---------------------------------------------------------------------------
PLATFORM_SIDE = 4 * 12 * INCH            # 4 ft square work surface, m
PLATFORM_HALF = np.array([PLATFORM_SIDE / 2, PLATFORM_SIDE / 2])   # 0.6096 m

# Top of the ground plane in item_sort.xml; the platform slab sits on it. Only
# used to work out how high a parked box rides.
GROUND_Z = -0.05


# ---------------------------------------------------------------------------
# The tool: two L brackets forming an adjustable jaw
# ---------------------------------------------------------------------------
# Under the carriage hang two right-angle brackets of 1/4 in stock, standing on
# edge and offset from each other along the (1, 1) diagonal. Their corners face
# each other, so between them they enclose a square pocket on all four sides:
# bracket A's two plates bound it in -x and -y, bracket B's in +x and +y.
#
#            B_y
#         +--------+          A is fixed to the carriage.
#         |        |B_x       B rides the jaw axis, a slide along the
#         |  box   |          diagonal, and closing it squeezes the box
#      A_x|        |          into A's corner. Both brackets push off
#         +--------+          either face, so with the jaw parked the
#            A_y              pair still shoves in +/-x and +/-y.
#
# Each plate spans BRACKET_WIDTH along its own long direction and reaches from
# just above the work surface to above the top of a box, so it catches a box low
# on the face and boxes slide instead of tipping.
BRACKET_THICKNESS = 0.25 * INCH    # 6.35 mm
BRACKET_BOTTOM_Z = 0.004           # m, just clear of the work surface
BRACKET_TOP_Z = 0.147              # m, above the top of a box (see BOX_HEIGHT)
BRACKET_HEIGHT = BRACKET_TOP_Z - BRACKET_BOTTOM_Z

# Jaw axis: the offset between the two brackets, measured along the diagonal, so
# this is the actuator's own stroke. Zero is mid-stroke, which is the convention
# every axis here follows (see TRAVEL). The pocket the brackets enclose is a
# square whose side is jaw_opening() below -- the diagonal stroke divided by
# root 2, since moving a distance q along (1,1)/sqrt(2) opens the pocket by
# q/sqrt(2) in each of x and y.
#
# The stroke is sized so the pocket spans every box in the pool: shut, it is a
# shade under a 3 in box; wide, a shade over a 12 in one.
JAW_TRAVEL = 0.17                  # m, half stroke -- must match the XML range
JAW_MID_OPENING = 0.20             # m, pocket side with the jaw axis at zero


def jaw_opening(offset):
    """Side of the square pocket (m) at a given jaw-axis position (m along the
    diagonal). This is the number that decides whether a box fits: a box of side
    s is captured when the opening exceeds s, and clamped when it is driven down
    to s."""
    return JAW_MID_OPENING + np.asarray(offset, dtype=float) / DIAG


JAW_OPENING_MIN = float(jaw_opening(-JAW_TRAVEL))   # 0.0798 m, jaw shut
JAW_OPENING_MAX = float(jaw_opening(+JAW_TRAVEL))   # 0.3202 m, jaw wide

# Width is a live GUI slider and sets all four plates at once: short arms grip a
# small box, long ones catch more of a big one's face.
#
# The ceiling is not arbitrary. Two L brackets that face each other can only
# close until their arms cross -- past that the plates would occupy the same
# space -- so the shut pocket can never be smaller than one arm. Keeping
# BRACKET_WIDTH_MAX at or under JAW_OPENING_MIN is what makes bracket-on-bracket
# contact impossible at any width and any offset, and it is the real trade the
# slider exposes: longer arms hold a big box better, shorter ones let the jaw
# shut on a small one.
BRACKET_WIDTH = 2.0 * INCH         # 0.0508 m, the default
BRACKET_WIDTH_MIN = 1.0 * INCH     # 0.0254 m
BRACKET_WIDTH_MAX = 3.0 * INCH     # 0.0762 m, just inside the shut opening

assert BRACKET_WIDTH_MAX <= JAW_OPENING_MIN, \
    "the brackets' arms would cross before the jaw reaches its stop"

# Where bracket A's corner sits, in -x and -y from the carriage centre. Set to
# half the widest opening so the tool's footprint is centred on the carriage
# with the jaw open: A's outer face then sits as far in -x as B's does in +x,
# which is what makes one symmetric TRAVEL cover both. Closing the jaw only pulls
# B back inside that envelope, so the widest opening is the worst case.
JAW_CORNER_OFFSET = JAW_OPENING_MAX / 2.0                  # 0.1601 m
BRACKET_A_CORNER = -JAW_CORNER_OFFSET                      # -0.1601 m, fixed
BRACKET_B_CORNER = -JAW_CORNER_OFFSET + JAW_MID_OPENING    # 0.0399 m, at jaw = 0

# How far the tool reaches from the carriage centre, worst case over every width
# and every offset: bracket A's outer face (or B's, they are equal by the choice
# above). This is what sizes the x/y travel.
TOOL_REACH = JAW_CORNER_OFFSET + BRACKET_THICKNESS         # 0.1665 m

# Bracket mass scales with width, since a longer arm is simply more of the same
# stock. This is deliberate: widening the brackets adds moving mass to x and y
# (and, for bracket B, to the jaw axis), so the width slider detunes all three a
# little and you can see it happen. 2.46 kg/m puts each arm of the default 2 in
# bracket at ~0.13 kg, so each bracket is ~0.25 kg and the whole tool ~0.5 kg.
BRACKET_MASS_PER_M = 2.46


def bracket_arm_mass(width):
    """Mass (kg) of ONE arm at a given width (m). A bracket is two of these and
    the tool is four; ItemSortSim sums them into each bracket's body mass."""
    return BRACKET_MASS_PER_M * float(width)


# ---------------------------------------------------------------------------
# Axes
# ---------------------------------------------------------------------------
# Order is fixed by the <actuator> list in item_sort.xml. Everything that
# indexes an axis (gains, forces, travel limits, the [x, y, jaw] state vector)
# uses this order.
AXIS_NAMES = ("axis_x", "axis_y", "axis_jaw")
AXIS_LABELS = ("X", "Y", "Jaw")
AXIS_UNITS = ("m", "m", "m")   # of a position / setpoint
EFFORT_UNITS = ("N", "N", "N")  # of a command
N_AXES = len(AXIS_NAMES)
X, Y, JAW = 0, 1, 2

# Half-travel of each axis (m), zero being mid-stroke. Must match the joint
# `range` in item_sort.xml -- ItemSortSim checks this at load time.
#
# x and y are equal because the platform is square, and both are set for the
# tool at its WORST case, not its current pose: 0.6096 (platform half) - 0.1665
# (TOOL_REACH) leaves 0.4431, so 0.44 keeps the brackets clear of the perimeter
# lip at every width and every jaw offset. Sizing travel to the current pose
# instead would mean re-deriving the soft limits every time a slider moved, and
# a real machine's limits are set for its widest tool too.
#
# The jaw's travel is its actuator's stroke along the diagonal, and unlike x and
# y it is not about clearance -- see JAW_TRAVEL for what sets it.
TRAVEL = np.array([0.44, 0.44, JAW_TRAVEL])

# Commanded targets stay this far off the hard end stops, so a little tracking
# overshoot cannot jam an axis against a limit (where the PID's integral winds up
# and the axis never reaches its setpoint). It matters most on the jaw, which is
# the one axis you would otherwise be tempted to park hard against its stop.
END_STOP_MARGIN = np.array([0.008, 0.008, 0.008])
SOFT_TRAVEL = TRAVEL - END_STOP_MARGIN

# What each actuator can physically deliver (N). Must match the actuator
# ctrlrange in item_sort.xml: ask for more and MuJoCo silently clips it, so
# anything above this is a lie to the controller. The jaw needs less than the
# slides -- it moves one bracket, and its job is to hold a box, not accelerate
# the bridge -- but not so little that it cannot grip.
MAX_FORCE = np.array([120.0, 80.0, 60.0])


# ---------------------------------------------------------------------------
# Drivetrain: what actually produces the force on each axis
# ---------------------------------------------------------------------------
# Each axis is a motor rigidly linked to what it moves through a pinion of pitch
# radius r. **None of this is in item_sort.xml.** MuJoCo still sees a plain force
# actuator on each slide joint; the drivetrain lives here as the conversion
# between the motor's world (shaft torque, rpm -- how a motor is specified and
# bought) and the axis's (N, m/s -- what the PID and the physics deal in):
#
#   F = tau / r        omega = v / r
#
# Both directions are the same single number per axis, `gear_gain` below: axis
# force per N*m of motor torque, and motor rad/s per m/s of axis speed. A rigid
# link cannot trade one without the other. Adding gear geometry to the model
# would buy nothing the controller can see, and would cost a stiff constraint in
# the solver.
#
# The default r puts the default motor torque exactly at MAX_FORCE, so out of
# the box the axes drive as hard as the actuators allow.
MOTOR_TORQUE = np.array([2.4, 1.6, 1.2])     # N*m at the shaft, per axis
MOTOR_TORQUE_MAX = np.array([6.0, 4.0, 3.0])  # slider ceilings

GEAR = np.array([0.020, 0.020, 0.020])       # pinion pitch radius, m
GEAR_MIN = np.array([0.005, 0.005, 0.005])
GEAR_MAX = np.array([0.060, 0.060, 0.060])

# The GUI reads radii in mm; SI everywhere else.
GEAR_LABEL = "gear radius"
GEAR_UNIT = "mm"
GEAR_SCALE = 1000.0                          # SI -> slider units

MOTOR_UNITS = ("N*m",) * N_AXES              # motor side, unlike EFFORT_UNITS
RPM_UNITS = ("rpm",) * N_AXES
RAD_S_TO_RPM = 60.0 / (2.0 * np.pi)


def gear_gain(gear):
    """Axis force per N*m of motor torque -- which is also motor rad/s per m/s of
    axis speed.

    `gear` is the per-axis pinion pitch radius in m, clamped into its usable
    range first so a radius can never reach zero and hand an axis infinite force.
    """
    return 1.0 / np.clip(np.asarray(gear, dtype=float), GEAR_MIN, GEAR_MAX)


def axis_effort_limit(torque, gear):
    """What the drivetrain can put on each axis (N), capped by what the actuator
    itself can deliver -- gearing for more force than MAX_FORCE just gets clipped
    by MuJoCo, silently, so cap it here where it is visible."""
    return np.minimum(np.asarray(torque, dtype=float) * gear_gain(gear), MAX_FORCE)


def motor_torque(effort, gear):
    """Shaft torque (N*m) behind a given axis force (N)."""
    return np.asarray(effort, dtype=float) / gear_gain(gear)


def motor_rpm(vel, gear):
    """Shaft speed (rev/min) at a given axis speed (m/s)."""
    return np.asarray(vel, dtype=float) * gear_gain(gear) * RAD_S_TO_RPM


# The limit the PID actually clamps its output to, per axis: whatever the motor
# and its gearing can deliver, never more than the actuator ceiling. The GUI
# drives this through the torque and gear sliders rather than setting it
# directly -- an axis is run soft by fitting a smaller motor or regearing it, not
# by wishing the force away.
FORCE_LIMIT = axis_effort_limit(MOTOR_TORQUE, GEAR)


# Dry (Coulomb) friction in each axis (N): the force the drive has to overcome
# before the axis moves at all, independent of speed. This is MuJoCo's joint
# `frictionloss`, and it is what a real rack, its bearings and its wipers
# actually cost you -- the thing that turns a clean PID into one that creeps,
# sticks short of target and hunts. The GUI exposes it per axis. Viscous drag
# (proportional to speed) is the joint `damping` in the XML.
AXIS_FRICTION = np.array([1.5, 0.8, 0.6])
AXIS_FRICTION_MAX = np.array([20.0, 20.0, 20.0])   # slider ceilings


# ---------------------------------------------------------------------------
# Loop rates
# ---------------------------------------------------------------------------
# One control loop: the position PID. SIM_HZ is a multiple of CTRL_HZ so every
# control tick lands on an exact physics-step boundary. Physics runs well above
# the loop rate because contact is the stiff part of this model, and the brackets
# are the reason it is this high: at 1/4 in thick, a step long enough to move one
# most of its own thickness can put it through the side of a box.
SIM_HZ = 1000     # physics; also written into model.opt.timestep at startup
CTRL_HZ = 100     # per-axis position PID

assert SIM_HZ % CTRL_HZ == 0, "SIM_HZ must be a multiple of CTRL_HZ"

SIM_DT = 1.0 / SIM_HZ
CTRL_DT = 1.0 / CTRL_HZ
SIM_PER_CTRL = SIM_HZ // CTRL_HZ


# ---------------------------------------------------------------------------
# PID gains (per axis, [x, y, jaw])
# ---------------------------------------------------------------------------
# Gains are in N/m and N/(m/s). Every axis is linear now, so unlike the earlier
# rotary-yaw machine these no longer carry mixed units -- only mixed scales.
#
# The x axis carries the whole bridge (~5.6 kg moving mass incl. armature), y the
# carriage and both brackets (~1.9 kg), and the jaw only bracket B (~0.7 kg).
# Roughly critically damped at ~2 Hz, with the joint damping already subtracted
# out of Kd. These are the primary tuning knobs, and the GUI exposes them as
# sliders so they can be trimmed live.
KP = np.array([760.0, 250.0, 110.0])
KD = np.array([115.0, 35.0, 12.0])
KI = np.array([220.0, 80.0, 40.0])

# Clamp on the integral state, per axis (m*s). Sized so the integral can still
# out-push axis friction at its slider maximum -- clamp it lower and a sticky
# axis parks short of its setpoint forever.
INTEGRAL_LIMIT = np.array([0.15, 0.15, 0.15])

# Slider ranges for the gain tuners in the GUI. Still per axis: each one needs
# roughly a third of the one above it for the same response, so a single shared
# range would waste most of the jaw's slider travel.
KP_MAX = np.array([2000.0, 800.0, 400.0])
KD_MAX = np.array([400.0, 150.0, 60.0])
KI_MAX = np.array([600.0, 300.0, 150.0])


# ---------------------------------------------------------------------------
# Boxes
# ---------------------------------------------------------------------------
# Every box is the same height and differs only in footprint -- 3, 6 or 12 in
# square. That is what a real infeed looks like: one carton height off the line,
# several floor sizes. It also means the brackets only have to be tall enough for
# one box height, which is what sets BRACKET_TOP_Z and the bridge ride height.
BOX_HEIGHT = 5.0 * INCH                  # 0.127 m
BOX_HALF_HEIGHT = BOX_HEIGHT / 2.0       # 0.0635 m

# Boxes are props: there is nothing sorting them, they are just there to shove
# and grip by hand. MuJoCo cannot add bodies to a live model, so the XML declares
# a fixed POOL of each size and the GUI "inserts" one by teleporting a spare
# from an off-platform staging row onto the surface. Nothing is on the platform
# until you put it there.
#
# Slot counts are lopsided on purpose: a 4 ft platform holds plenty of 3 in
# boxes and only a handful of 12 in ones.
#
# Every size fits the jaw, and every size can be clamped: the largest is 0.3048
# m across against a widest opening of 0.3202, and the smallest is 0.0762
# against a shut opening of 0.0798.
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
# slider. This is the number that decides whether the brackets shove a box or
# just skid it, and how far a box coasts after they stop -- so it is a knob, not
# a constant. item_sort.xml pins it with explicit contact <pair>s (platform
# <-> each box geom) rather than leaving it to MuJoCo's "max of the two geoms"
# rule, so the value here is the coefficient that actually applies.
SURFACE_FRICTION = 0.5
SURFACE_FRICTION_MIN = 0.02   # near-frictionless: boxes glide and overshoot
SURFACE_FRICTION_MAX = 1.5    # grippy: the axis saturates before a box moves

# Clearance a freshly inserted box keeps from the walls, the tool and every box
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


def clamp_to_travel(xyj):
    """Clamp an (x, y, jaw) axis target into the usable travel (soft limits)."""
    return np.clip(np.asarray(xyj, dtype=float), -SOFT_TRAVEL, SOFT_TRAVEL)

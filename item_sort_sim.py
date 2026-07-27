import numpy as np
import mujoco

from item_sort_config import (
    TRAVEL, PLATFORM_HALF, BOX_SPAWN_MARGIN, box_park_pose, paddle_mass,
)


def _box_inertia(half_extents, mass):
    """Diagonal inertia of a solid cuboid about its own centre, MuJoCo's
    (Ixx, Iyy, Izz) order. `half_extents` is a geom `size`, i.e. half the side
    lengths, which is why this is m/3 * (b^2 + c^2) and not m/12 * (2b)^2 ..."""
    a, b, c = (float(v) for v in half_extents)
    return float(mass) / 3.0 * np.array([b * b + c * c,
                                         a * a + c * c,
                                         a * a + b * b])


class ItemSortSim:
    """MuJoCo stand-in for the sorting gantry: its three axes, the blade on the
    end of them, and the pool of boxes that can be put on the platform.

    Positions, velocities and commands are 3-vectors in AXIS_NAMES order
    ([x, y, yaw]) and carry that axis's units -- m / m/s / N on the two slides,
    rad / rad/s / N*m on the hinge. As in MuJoCo itself, "force" names the
    command on every axis regardless of joint type. There is no
    radian<->revolution conversion anywhere: unlike the balance_bot / biped
    wrappers this project stays in plain SI.

    Boxes work by pool, not by creation: MuJoCo models are fixed once compiled,
    so the XML declares every box that could ever appear and parks the unused
    ones off-platform. spawn_box brings one in, park_box / park_all_boxes send
    them back, and a "slot" throughout is an index into `box_names`.

    Everything the GUI can dial at runtime -- blade width, box masses, the
    box/platform friction, the axes' dry friction -- is a *model* field rather
    than a constant, so the setters below write mjModel in place. Each one
    ignores a write that would not change anything, since some of them (mass or
    size changes) mean recomputing derived model constants and the GUI calls them
    every tick.
    """

    def __init__(self, model_path, axis_names, box_names=()):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.axis_names = list(axis_names)
        self.n = len(self.axis_names)

        self.actuator_ids = []
        for name in self.axis_names:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if aid < 0:
                raise ValueError(f"actuator '{name}' not found in {model_path}")
            self.actuator_ids.append(aid)
        self.actuator_ids = np.array(self.actuator_ids)

        qpos_addrs, qvel_addrs, ranges = [], [], []
        for name in self.axis_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"joint '{name}' not found in {model_path}")
            qpos_addrs.append(self.model.jnt_qposadr[jid])
            qvel_addrs.append(self.model.jnt_dofadr[jid])
            ranges.append(self.model.jnt_range[jid])
        self.qpos_addrs = np.array(qpos_addrs)
        self.qvel_addrs = np.array(qvel_addrs)
        self.travel_limits = np.array(ranges)   # (n, 2) [lo, hi] per axis

        # The XML joint ranges *are* the machine's travel; the planner clamps
        # targets against config.TRAVEL, so the two must agree.
        if not np.allclose(self.travel_limits[:, 1], TRAVEL, atol=1e-9) or \
           not np.allclose(self.travel_limits[:, 0], -TRAVEL, atol=1e-9):
            raise ValueError(
                f"axis travel in {model_path} {self.travel_limits.tolist()} does "
                f"not match item_sort_config.TRAVEL {TRAVEL.tolist()}")

        # The blade: the gantry's only colliding geom, and the one piece of
        # geometry the GUI reshapes at runtime.
        self.paddle_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "paddle")
        if self.paddle_geom_id < 0:
            raise ValueError(f"geom 'paddle' not found in {model_path}")
        self.paddle_body_id = int(self.model.geom_bodyid[self.paddle_geom_id])

        # Box bodies -- the pool of items to shove around. Optional: an empty
        # list gives a bare gantry, useful for tuning the axis PID on its own.
        self.box_names = list(box_names)
        self.box_body_ids, self.box_geom_ids = [], []
        self.box_qpos_addrs, self.box_dof_addrs = [], []
        self.box_halfs, self.box_half_heights = [], []
        for name in self.box_names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                raise ValueError(f"body '{name}' not found in {model_path}")
            jid = self.model.body_jntadr[bid]
            if jid < 0:
                raise ValueError(f"box body '{name}' in {model_path} has no joint; "
                                 "each box needs a freejoint so it can be moved")
            gid = self.model.body_geomadr[bid]
            self.box_body_ids.append(bid)
            self.box_geom_ids.append(gid)
            self.box_qpos_addrs.append(self.model.jnt_qposadr[jid])
            self.box_dof_addrs.append(self.model.jnt_dofadr[jid])
            # Size straight out of the model: the XML stays the authority on how
            # big a box actually is. Footprint half-width and half-height are
            # separate now -- every box is the same height and differs only in
            # its footprint.
            self.box_halfs.append(float(self.model.geom_size[gid][0]))
            self.box_half_heights.append(float(self.model.geom_size[gid][2]))
        self.box_body_ids = np.array(self.box_body_ids, dtype=int)
        self.box_geom_ids = np.array(self.box_geom_ids, dtype=int)
        self.box_qpos_addrs = np.array(self.box_qpos_addrs, dtype=int)
        self.box_dof_addrs = np.array(self.box_dof_addrs, dtype=int)
        self.box_halfs = np.array(self.box_halfs, dtype=float)
        self.box_half_heights = np.array(self.box_half_heights, dtype=float)
        # Which slots are currently on the platform (the rest are parked).
        self._on_platform = np.zeros(len(self.box_body_ids), dtype=bool)

        # Predefined platform<->box contacts, whose sliding coefficient *is* the
        # box/platform friction (see the <contact> block in the XML).
        platform_gid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "platform")
        self.surface_pair_ids = np.array(
            [i for i in range(self.model.npair)
             if platform_gid in (self.model.pair_geom1[i], self.model.pair_geom2[i])],
            dtype=int)

        # Last value written by each live-tunable setter, so repeat calls with an
        # unchanged value are free.
        self._applied = {}

        # Optional "home" keyframe -- the model's nominal initial state. This
        # model has none (its bodies already declare it), but sibling models do.
        self._home_key = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if self._home_key < 0:
            self._home_key = None

        self._force = np.zeros(self.n)
        self.reset_state()

    @property
    def sim_dt(self):
        return self.model.opt.timestep

    def reset_state(self, pos=None, vel=None):
        """Reset to the nominal initial state: axes centred, blade square on, and
        every box back in the staging row, i.e. a clear platform. `pos`/`vel` are
        optional axis overrides in AXIS_NAMES order applied on top."""
        if self._home_key is not None:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self._home_key)
        else:
            mujoco.mj_resetData(self.model, self.data)
        if pos is not None:
            self.data.qpos[self.qpos_addrs] = np.asarray(pos, dtype=float)
        if vel is not None:
            self.data.qvel[self.qvel_addrs] = np.asarray(vel, dtype=float)
        self._force = np.zeros(self.n)
        self.park_all_boxes()   # also does the mj_forward

    def set_force(self, force):
        """Command axis efforts, [x, y, yaw] -- N on the slides, N*m on the
        hinge. Held until the next call."""
        self._force = np.asarray(force, dtype=float)

    def step_n(self, n):
        """Advance the sim by n physics steps, holding the current force."""
        for _ in range(n):
            self.data.ctrl[self.actuator_ids] = self._force
            mujoco.mj_step(self.model, self.data)

    def get_pos(self):
        """Axis positions [x, y, yaw] -- m, m, rad."""
        return self.data.qpos[self.qpos_addrs].copy()

    def get_vel(self):
        """Axis velocities [x, y, yaw] -- m/s, m/s, rad/s."""
        return self.data.qvel[self.qvel_addrs].copy()

    def stop(self):
        self._force = np.zeros(self.n)
        self.data.ctrl[self.actuator_ids] = 0.0

    # ---- live-tunable model fields -----------------------------------------

    def set_surface_friction(self, mu):
        """Sliding coefficient between the boxes and the platform.

        Writes the predefined platform<->box pairs, which is the value that
        actually governs a box sliding on the surface. Box geoms are set to match
        as well, so box-on-box and box-against-the-lip stay consistent with the
        surface instead of holding a stale default.
        """
        mu = float(mu)
        if self._applied.get("mu") == mu:
            return
        self._applied["mu"] = mu
        if len(self.surface_pair_ids):
            self.model.pair_friction[self.surface_pair_ids, 0:2] = mu
        if len(self.box_geom_ids):
            self.model.geom_friction[self.box_geom_ids, 0] = mu

    def set_axis_friction(self, values):
        """Dry (Coulomb) friction per axis, [x, y, yaw] in N, N, N*m -- MuJoCo
        frictionloss.

        The effort each axis must overcome before it moves at all, regardless of
        speed. Viscous drag is the joints' `damping` and is not tunable here.
        """
        values = np.asarray(values, dtype=float)
        if np.array_equal(self._applied.get("axis_friction"), values):
            return
        self._applied["axis_friction"] = values.copy()
        self.model.dof_frictionloss[self.qvel_addrs] = values

    def set_box_mass(self, slots, mass):
        """Mass (kg) of every box in `slots` -- one call per box size.

        Inertia is recomputed to match: MuJoCo takes body_mass and body_inertia
        as independent inputs, so changing the mass alone would leave a box with
        the rotational inertia of its old one and make it tip strangely.
        """
        mass = float(mass)
        slots = tuple(slots)
        if self._applied.get(("mass", slots)) == mass:
            return
        self._applied[("mass", slots)] = mass
        for slot in slots:
            bid = self.box_body_ids[slot]
            self.model.body_mass[bid] = mass
            self.model.body_inertia[bid] = _box_inertia(
                self.model.geom_size[self.box_geom_ids[slot]], mass)
        # Refresh the constants MuJoCo derives from the mass properties
        # (subtree masses, the reference mass matrix diagonal).
        mujoco.mj_setConst(self.model, self.data)

    def set_paddle_width(self, width):
        """Blade width (m): its extent along the local y of the yaw axis.

        Reshapes the geom rather than swapping models, so besides the size this
        has to maintain everything the compiler derived from it: the collision
        bounding volumes (stale ones would let the broad-phase miss contacts as
        the blade grows) and the blade's mass and inertia, which scale with width
        because a wider blade is more of the same stock.
        """
        width = float(width)
        if self._applied.get("paddle_width") == width:
            return
        self._applied["paddle_width"] = width

        gid, bid = self.paddle_geom_id, self.paddle_body_id
        self.model.geom_size[gid, 1] = width / 2.0
        size = self.model.geom_size[gid]
        self.model.geom_rbound[gid] = float(np.linalg.norm(size))
        self.model.geom_aabb[gid, 0:3] = 0.0      # box is centred in its frame
        self.model.geom_aabb[gid, 3:6] = size

        mass = paddle_mass(width)
        self.model.body_mass[bid] = mass
        self.model.body_inertia[bid] = _box_inertia(size, mass)
        mujoco.mj_setConst(self.model, self.data)

    def get_paddle_width(self):
        """Current blade width, m."""
        return 2.0 * float(self.model.geom_size[self.paddle_geom_id, 1])

    # ---- box pool ----------------------------------------------------------

    def is_box_active(self, slot):
        """True if box `slot` is on the platform rather than parked."""
        return bool(self._on_platform[slot])

    def active_slots(self):
        """Slots currently on the platform."""
        return np.flatnonzero(self._on_platform)

    def spawn_box(self, slots):
        """Put the first spare box from `slots` down on the platform.

        Lands it on the emptiest patch of surface -- as far as possible from the
        walls, the blade and every box already down -- so inserting a run of
        boxes spreads them out instead of stacking them. Returns the slot used,
        or None if `slots` holds no spare or nothing of that size still fits.
        """
        for slot in slots:
            if self._on_platform[slot]:
                continue
            spot = self._free_spot(self.box_halfs[slot])
            if spot is None:
                return None     # a spare exists, but there is no room for it
            self._place_box(slot, np.array([spot[0], spot[1],
                                            self.box_half_heights[slot]]))
            self._on_platform[slot] = True
            return slot
        return None

    def park_box(self, slot):
        """Send box `slot` back to the staging row, off the platform."""
        self._place_box(slot, box_park_pose(slot))
        self._on_platform[slot] = False

    def park_all_boxes(self):
        """Clear the platform: every box back to the staging row."""
        for slot in range(len(self.box_body_ids)):
            self.park_box(slot)

    def _place_box(self, slot, xyz):
        """Teleport a box to `xyz`, upright and at rest."""
        addr = self.box_qpos_addrs[slot]
        self.data.qpos[addr:addr + 3] = xyz
        self.data.qpos[addr + 3:addr + 7] = (1.0, 0.0, 0.0, 0.0)   # identity quat
        dof = self.box_dof_addrs[slot]
        self.data.qvel[dof:dof + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _free_spot(self, half, grid=25):
        """Best (x, y) on the platform for a box of footprint half-width `half`,
        or None if nothing fits.

        Scores a grid of candidate centres by their clearance -- the smallest gap
        to any wall, to the blade, or to a box already down -- and takes the
        roomiest. Boxes are axis-aligned squares, so gaps are Chebyshev distances
        between centres less the two half-extents. The blade is not axis-aligned
        once yaw is off zero, so it stands in as a square of its longer side:
        crude, but it only ever refuses a spot that was marginal anyway.
        """
        reach = PLATFORM_HALF - half - BOX_SPAWN_MARGIN
        if np.any(reach <= 0):
            return None     # box is wider than the platform

        blade_half = float(np.max(self.model.geom_size[self.paddle_geom_id, 0:2]))
        obstacles = [(self.get_paddle_pos(), blade_half)]
        for slot in self.active_slots():
            obstacles.append((self.data.xpos[self.box_body_ids[slot], :2],
                              self.box_halfs[slot]))

        best, best_clear = None, -np.inf
        for cx in np.linspace(-reach[0], reach[0], grid):
            for cy in np.linspace(-reach[1], reach[1], grid):
                centre = np.array([cx, cy])
                clear = np.min(reach - np.abs(centre))
                for pos, other_half in obstacles:
                    gap = np.max(np.abs(centre - pos)) - (half + other_half)
                    clear = min(clear, gap)
                if clear > best_clear:
                    best_clear, best = clear, centre
        return best if best_clear > BOX_SPAWN_MARGIN else None

    # ---- box / perception helpers (sim stand-in for an overhead camera) -----

    def get_box_positions(self):
        """Centres of every box in the pool as an (n_boxes, 2) array of (x, y),
        m. Parked boxes are included and read far off the platform; filter with
        active_slots() for just what a camera over the surface would see."""
        if len(self.box_body_ids) == 0:
            return np.zeros((0, 2))
        return self.data.xpos[self.box_body_ids, :2].copy()

    def get_box_heights(self):
        """Box centre heights, m. A box that has been tipped or climbed onto
        another reads noticeably off its nominal half-height."""
        if len(self.box_body_ids) == 0:
            return np.zeros(0)
        return self.data.xpos[self.box_body_ids, 2].copy()

    def get_paddle_pos(self):
        """Blade centre (x, y), m -- the axes' intersection, which is also the
        yaw axis, so this is independent of the blade's angle."""
        return self.data.qpos[self.qpos_addrs[:2]].copy()

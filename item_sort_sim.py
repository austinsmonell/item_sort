import numpy as np
import mujoco

from item_sort_config import TRAVEL


class ItemSortSim:
    """MuJoCo stand-in for the sorting gantry: its two linear axes plus the
    boxes sitting on the platform.

    Units are the linear SI convention used across this project (m, m/s, N), so
    unlike the balance_bot / biped wrappers there is no radian<->revolution
    conversion: the slide joints are already in metres.

    get_pos / get_vel / set_force all operate on 2-vectors in AXIS_NAMES order
    ([x, y]). The box helpers are a sim stand-in for whatever perception a real
    machine would use (overhead camera, fiducials); nothing drives off them yet,
    they are there for when something above the PID needs to see the boxes.
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
        self.travel_limits = np.array(ranges)   # (n, 2) [lo, hi] per axis, m

        # The XML joint ranges *are* the machine's travel; the planner clamps
        # targets against config.TRAVEL, so the two must agree.
        if not np.allclose(self.travel_limits[:, 1], TRAVEL, atol=1e-9) or \
           not np.allclose(self.travel_limits[:, 0], -TRAVEL, atol=1e-9):
            raise ValueError(
                f"axis travel in {model_path} {self.travel_limits.tolist()} does "
                f"not match item_sort_config.TRAVEL {TRAVEL.tolist()}")

        # Box bodies -- the items being sorted. Optional: an empty list gives a
        # bare gantry, useful for tuning the axis PID on its own.
        self.box_names = list(box_names)
        self.box_body_ids = []
        for name in self.box_names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                raise ValueError(f"body '{name}' not found in {model_path}")
            self.box_body_ids.append(bid)
        self.box_body_ids = np.array(self.box_body_ids, dtype=int)

        # Optional "home" keyframe -- the model's nominal initial state (gantry
        # centred, boxes at their start poses). When present, reset_state
        # initializes to it instead of the bare model default.
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
        """Reset to the nominal initial state (the 'home' keyframe if the model
        defines one, else the bare default). `pos`/`vel` are optional axis
        overrides (m, m/s) in AXIS_NAMES order applied on top; the boxes keep
        their keyframe poses."""
        if self._home_key is not None:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self._home_key)
        else:
            mujoco.mj_resetData(self.model, self.data)
        if pos is not None:
            self.data.qpos[self.qpos_addrs] = np.asarray(pos, dtype=float)
        if vel is not None:
            self.data.qvel[self.qvel_addrs] = np.asarray(vel, dtype=float)
        self._force = np.zeros(self.n)
        mujoco.mj_forward(self.model, self.data)

    def set_force(self, force):
        """Command axis forces (N), [x, y]. Held until the next call."""
        self._force = np.asarray(force, dtype=float)

    def step_n(self, n):
        """Advance the sim by n physics steps, holding the current force."""
        for _ in range(n):
            self.data.ctrl[self.actuator_ids] = self._force
            mujoco.mj_step(self.model, self.data)

    def get_pos(self):
        """Axis positions [x, y], m."""
        return self.data.qpos[self.qpos_addrs].copy()

    def get_vel(self):
        """Axis velocities [x, y], m/s."""
        return self.data.qvel[self.qvel_addrs].copy()

    def stop(self):
        self._force = np.zeros(self.n)
        self.data.ctrl[self.actuator_ids] = 0.0

    # ---- box / perception helpers (sim stand-in for an overhead camera) -----

    def get_box_positions(self):
        """Box centres on the platform as an (n_boxes, 2) array of (x, y), m."""
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
        """Paddle centre (x, y), m. Identical to get_pos() for this kinematic
        chain -- the paddle hangs on the axes' intersection."""
        return self.get_pos()

import numpy as np

from item_sort_config import N_AXES, MAX_FORCE, clamp_to_travel


class AxisPID:
    """Per-axis position PID with an integral clamp.

    The gantry's only control loop: drives the paddle to a target (x, y) and
    holds it there. `target`, `kp`, `kd`, `ki` may be scalars (shared by both
    axes) or 2-vectors; everything broadcasts over the axis vector. I/O is in
    the linear convention (m, m/s, N), matching ItemSortSim.

    Gains are plain attributes so they can be retuned live from the GUI.
    """

    def __init__(self, target, kp, kd, ki, dt, integral_limit, max_force,
                 n_axes=N_AXES):
        self.n_axes = n_axes
        self.target = clamp_to_travel(target)
        self.kp = self._vec(kp)
        self.kd = self._vec(kd)
        self.ki = self._vec(ki)
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral = np.zeros(n_axes)
        self.set_max_force(max_force)

    def _vec(self, v):
        return np.broadcast_to(np.asarray(v, float), (self.n_axes,)).copy()

    def set_target(self, xy):
        """Command a new paddle position, clamped into the usable travel."""
        self.target = clamp_to_travel(xy)

    def set_gains(self, kp=None, kd=None, ki=None):
        if kp is not None:
            self.kp = self._vec(kp)
        if kd is not None:
            self.kd = self._vec(kd)
        if ki is not None:
            self.ki = self._vec(ki)

    def set_max_force(self, max_force):
        """Set the per-axis output clamp (N), never above what the actuator can
        actually deliver -- asking for more just gets clipped downstream."""
        self.max_force = np.minimum(self._vec(max_force), MAX_FORCE)

    def compute(self, pos, vel):
        """Force (N) for each axis, [x, y]."""
        pos = np.asarray(pos, dtype=float)
        vel = np.asarray(vel, dtype=float)
        err = self.target - pos
        self.integral = np.clip(self.integral + err * self.dt,
                                -self.integral_limit, self.integral_limit)
        u = self.kp * err + self.ki * self.integral - self.kd * vel
        return np.clip(u, -self.max_force, self.max_force)

    def reset(self):
        """Drop the integral state (after a jump in target, or a sim reset)."""
        self.integral = np.zeros(self.n_axes)

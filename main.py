"""Run harness: a MuJoCo viewer plus a Tk panel that commands the paddle.

Two windows come up. The MuJoCo viewer shows the gantry; the control panel has
an X/Y slider pair (the PID setpoint, in metres) and per-axis Kp/Kd/Ki sliders
so gains can be trimmed while it runs.

The sim and the GUI share one thread, and **Tk owns it**: `root.mainloop()` runs,
and physics advances in a `root.after` callback that steps up to the wall clock.
That keeps the PID reading slider values directly with no locking, and -- unlike
driving Tk from the sim loop with periodic `root.update()` calls -- it leaves
mouse events serviced continuously, so a slider follows the pointer instead of
jumping once per redraw.
"""

import time
import tkinter as tk

import numpy as np

from item_sort_config import (
    AXIS_NAMES, BOX_NAMES, N_AXES, SOFT_TRAVEL,
    KP, KD, KI, KP_MAX, KD_MAX, KI_MAX, INTEGRAL_LIMIT,
    MAX_FORCE, FORCE_LIMIT,
    SIM_DT, CTRL_DT, SIM_PER_CTRL,
)
from controller import AxisPID


start_target = np.array([0.0, 0.0])   # where the paddle parks at startup
print_period = 0.5     # s, console print period (0 disables)

# How often Tk reschedules a physics batch. 5 ms is well under Tk's own event
# servicing, so the loop keeps up with real time while the pointer stays smooth.
TICK_MS = 5
# Ceiling on one batch, so a hitch (window drag, slow frame) is absorbed rather
# than repaid in one long uninterruptible burst that freezes the GUI.
MAX_CATCHUP_STEPS = 50


class ControlPanel:
    """Tk window holding the setpoint sliders and the live gain tuners."""

    def __init__(self, pid):
        self.pid = pid
        self.root = tk.Tk()
        self.root.title("item_sort — paddle control")
        self.alive = True
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # --- setpoint ---
        target = tk.LabelFrame(self.root, text="Paddle target (m)", padx=6, pady=4)
        target.pack(fill="x", padx=8, pady=(8, 4))
        self.target_vars = []
        for axis, name in enumerate("XY"):
            var = tk.DoubleVar(value=float(pid.target[axis]))
            limit = float(SOFT_TRAVEL[axis])
            tk.Scale(target, from_=-limit, to=limit, resolution=0.001,
                     orient="horizontal", length=380, variable=var,
                     label=f"{name}   (±{limit:.3f})").pack(fill="x")
            self.target_vars.append(var)

        # --- gains + force limit, per axis ---
        self.gain_vars = {}
        for axis, name in enumerate("XY"):
            frame = tk.LabelFrame(self.root, text=f"{name} axis", padx=6, pady=4)
            frame.pack(fill="x", padx=8, pady=4)
            # Force limit tops out at MAX_FORCE: the actuator ctrlrange clips
            # anything beyond it anyway.
            for label, value, top in (("Kp", KP[axis], KP_MAX),
                                      ("Kd", KD[axis], KD_MAX),
                                      ("Ki", KI[axis], KI_MAX),
                                      ("Fmax", FORCE_LIMIT[axis], MAX_FORCE[axis])):
                var = tk.DoubleVar(value=float(value))
                tk.Scale(frame, from_=0.0, to=float(top), resolution=top / 400.0,
                         orient="horizontal", length=380, variable=var,
                         label=f"{label} (N)" if label == "Fmax" else label,
                         ).pack(fill="x")
                self.gain_vars[(axis, label)] = var

        # --- buttons + readout ---
        row = tk.Frame(self.root)
        row.pack(fill="x", padx=8, pady=(0, 4))
        tk.Button(row, text="Centre", command=self.centre).pack(side="left")
        tk.Button(row, text="Reset integral",
                  command=pid.reset).pack(side="left", padx=6)

        self.readout = tk.Label(self.root, font=("Consolas", 10), justify="left",
                                anchor="w")
        self.readout.pack(fill="x", padx=10, pady=(0, 8))

    def centre(self):
        for var in self.target_vars:
            var.set(0.0)

    def _on_close(self):
        self.close()

    def close(self):
        """Tear the window down; safe to call twice, or after Tk is already gone."""
        if not self.alive:
            return
        self.alive = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def apply_to(self, pid):
        """Push the current slider values into the PID."""
        pid.set_target([var.get() for var in self.target_vars])
        pid.set_gains(
            kp=[self.gain_vars[(a, "Kp")].get() for a in range(N_AXES)],
            kd=[self.gain_vars[(a, "Kd")].get() for a in range(N_AXES)],
            ki=[self.gain_vars[(a, "Ki")].get() for a in range(N_AXES)],
        )
        pid.set_max_force([self.gain_vars[(a, "Fmax")].get()
                           for a in range(N_AXES)])

    def refresh(self, pos, force):
        """Redraw the readout. Tk pumps its own events -- see run_in_sim."""
        if not self.alive:
            return
        err = self.pid.target - pos
        # Flag saturation: at the limit the loop is open, and no amount of gain
        # tuning explains what the paddle does next.
        sat = np.abs(force) >= self.pid.max_force - 1e-6
        self.readout.config(text=(
            f"pos  [{pos[0]:+.3f} {pos[1]:+.3f}] m\n"
            f"err  [{err[0]:+.3f} {err[1]:+.3f}] m\n"
            f"F    [{force[0]:+6.1f} {force[1]:+6.1f}] N"
            + ("   SAT " + " ".join("XY"[a] for a in range(N_AXES) if sat[a])
               if sat.any() else "")))


def run_in_sim():
    import mujoco.viewer
    from item_sort_sim import ItemSortSim

    robot = ItemSortSim("item_sort.xml", AXIS_NAMES, BOX_NAMES)
    # Drive physics at SIM_HZ so the control loop lands on exact step counts.
    robot.model.opt.timestep = SIM_DT
    robot.reset_state()

    pid = AxisPID(target=start_target, kp=KP, kd=KD, ki=KI, dt=CTRL_DT,
                  integral_limit=INTEGRAL_LIMIT, max_force=FORCE_LIMIT)
    panel = ControlPanel(pid)

    print_every = int(round(print_period / SIM_DT)) if print_period else 0

    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
        # Look down on the platform -- the useful view for a planar gantry.
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -60
        viewer.cam.distance = 2.0

        sim_start = robot.data.time
        wall_start = time.perf_counter()
        loop = {"tick": 0, "force": np.zeros(N_AXES)}

        def step_batch():
            """Advance physics up to the wall clock, then redraw. Rescheduled by
            Tk, so mouse events are serviced between batches instead of waiting
            on a blind root.update() -- that is what makes the sliders track the
            pointer while the sim runs."""
            if not (viewer.is_running() and panel.alive):
                panel.close()
                return

            panel.apply_to(pid)

            # How many steps are owed. Capped so a stall (a dragged window, a
            # slow frame) cannot spiral into a long uninterruptible catch-up.
            behind = (time.perf_counter() - wall_start) - (robot.data.time - sim_start)
            owed = min(int(behind / SIM_DT), MAX_CATCHUP_STEPS)
            pos = robot.get_pos()
            for _ in range(max(owed, 0)):
                pos, vel = robot.get_pos(), robot.get_vel()
                if loop["tick"] % SIM_PER_CTRL == 0:
                    loop["force"] = pid.compute(pos, vel)
                    robot.set_force(loop["force"])
                robot.step_n(1)   # one physics step at SIM_HZ
                loop["tick"] += 1
                if print_every and loop["tick"] % print_every == 0:
                    print(f"t={robot.data.time - sim_start:6.2f}s  "
                          f"xy=[{pos[0]:+.3f} {pos[1]:+.3f}]  "
                          f"tgt=[{pid.target[0]:+.3f} {pid.target[1]:+.3f}]  "
                          f"F=[{loop['force'][0]:+6.1f} {loop['force'][1]:+6.1f}]N")

            viewer.sync()
            panel.refresh(pos, loop["force"])
            panel.root.after(TICK_MS, step_batch)

        try:
            panel.root.after(0, step_batch)
            panel.root.mainloop()   # Tk owns the thread; step_batch runs inside it
        finally:
            robot.stop()
            panel.close()


def run_on_hardware():
    """Placeholder — no sorting-gantry hardware yet.

    When a physical machine exists, mirror balance_bot/pendulum: open the axis
    drive backend, home both axes against their end stops to establish the zero
    that item_sort_config.TRAVEL is measured from, then run pid.compute(pos, vel)
    at CTRL_HZ, writing the resulting force (or its current-loop equivalent) to
    each axis. Ensure a safe stop -- zero force, brakes engaged -- on exit.
    """
    raise NotImplementedError("item_sort hardware is not implemented yet")


if __name__ == "__main__":
    try:
        run_in_sim()
    except KeyboardInterrupt:
        pass

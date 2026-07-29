"""Run harness: a MuJoCo viewer plus a Tk panel that commands the gantry.

Two windows come up. The MuJoCo viewer shows the machine; the control panel has

  * an X / Y / Yaw slider set -- the PID setpoint, in m, m and rad
  * per-axis Kp/Kd/Ki, motor torque, gearing and dry-friction sliders, so the
    loop and the drivetrain it is fighting can both be trimmed while it runs,
    plus a motor-side readout (torque, rpm, and a resettable peak torque)
  * a blade width slider
  * buttons that drop 3 / 6 / 12 in boxes onto the platform (it starts empty), a
    mass slider per box size, and the box-on-platform friction coefficient

The sim and the GUI share one thread, and **Tk owns it**: `root.mainloop()` runs,
and physics advances in a `root.after` callback that steps up to the wall clock.
That keeps the PID reading slider values directly with no locking, and -- unlike
driving Tk from the sim loop with periodic `root.update()` calls -- it leaves
mouse events serviced continuously, so a slider follows the pointer instead of
jumping once per redraw. Button callbacks land between physics batches, never
inside one, which is what lets them write box poses straight into the sim.
"""

import time
import tkinter as tk

import numpy as np

from item_sort_config import (
    AXIS_NAMES, AXIS_LABELS, AXIS_UNITS, EFFORT_UNITS, BOX_NAMES, N_AXES,
    SOFT_TRAVEL, KP, KD, KI, KP_MAX, KD_MAX, KI_MAX, INTEGRAL_LIMIT,
    MAX_FORCE, FORCE_LIMIT, AXIS_FRICTION, AXIS_FRICTION_MAX,
    MOTOR_TORQUE, MOTOR_TORQUE_MAX, MOTOR_UNITS, RPM_UNITS,
    GEAR, GEAR_MIN, GEAR_MAX, GEAR_LABELS, GEAR_UNITS, GEAR_SCALE,
    gear_gain, axis_effort_limit, motor_torque, motor_rpm,
    PADDLE_WIDTH, PADDLE_WIDTH_MIN, PADDLE_WIDTH_MAX,
    BOX_KINDS, BOX_SIDE_IN, BOX_SLOTS, BOX_KIND_SLOTS,
    BOX_MASS, BOX_MASS_MIN, BOX_MASS_MAX,
    SURFACE_FRICTION, SURFACE_FRICTION_MIN, SURFACE_FRICTION_MAX,
    SIM_DT, CTRL_DT, SIM_PER_CTRL,
)
from controller import AxisPID


start_target = np.zeros(N_AXES)   # where the blade parks at startup
print_period = 0.5     # s, console print period (0 disables)

# How often Tk reschedules a physics batch. 5 ms is well under Tk's own event
# servicing, so the loop keeps up with real time while the pointer stays smooth.
TICK_MS = 5
# Ceiling on one batch, so a hitch (window drag, slow frame) is absorbed rather
# than repaid in one long uninterruptible burst that freezes the GUI.
MAX_CATCHUP_STEPS = 100


def _axis_row(name, values, units, width=7, precision=3):
    """One line of the readout: a label then a value per axis, with units."""
    cells = "  ".join(f"{AXIS_LABELS[a]} {values[a]:+{width}.{precision}f} {units[a]}"
                      for a in range(N_AXES))
    return f"{name:5s}{cells}"


class ControlPanel:
    """Tk window holding the setpoint sliders, the live gain tuners, and -- when
    given a sim to talk to -- the blade width, box pool and friction knobs.

    `robot` is optional: without it the panel is just the axis controls, which is
    what a hardware run would want (no boxes to insert, and the blade and the
    friction are whatever the real machine has).
    """

    def __init__(self, pid, robot=None):
        self.pid = pid
        self.robot = robot
        self.root = tk.Tk()
        self.root.title("item_sort — gantry control")
        self.alive = True
        # Highest shaft torque each motor has been asked for since the last
        # reset. Peak-hold rather than instantaneous, because the moment that
        # sizes a motor -- breaking a heavy box loose, catching an overshoot --
        # is over long before the readout redraws.
        self.peak_torque = np.zeros(N_AXES)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # --- setpoint, one slider per axis (m, m, rad) ---
        target = tk.LabelFrame(self.root, text="Blade target", padx=6, pady=4)
        target.pack(fill="x", padx=8, pady=(8, 4))
        self.target_vars = []
        for axis, label in enumerate(AXIS_LABELS):
            var = tk.DoubleVar(value=float(pid.target[axis]))
            limit = float(SOFT_TRAVEL[axis])
            unit = AXIS_UNITS[axis]
            tk.Scale(target, from_=-limit, to=limit, resolution=0.001,
                     orient="horizontal", length=420, variable=var,
                     label=f"{label}   ({unit}, ±{limit:.3f})").pack(fill="x")
            self.target_vars.append(var)

        # --- gains, drivetrain and axis friction: one column per axis ---
        # Side by side rather than stacked, because three axes' worth of sliders
        # in one column is taller than a screen.
        self.gain_vars = {}
        columns = tk.Frame(self.root)
        columns.pack(fill="x", padx=8, pady=4)
        for axis, label in enumerate(AXIS_LABELS):
            unit = EFFORT_UNITS[axis]
            frame = tk.LabelFrame(columns, text=f"{label} axis", padx=4, pady=4)
            frame.pack(side="left", fill="both", expand=True, padx=2)
            # The PID's output limit is not a slider: it is what the motor and
            # its gearing can deliver, so the knobs are the motor's stall torque
            # and the pitch radius (ratio, on yaw) it drives through -- size the
            # drivetrain and the force limit follows. "friction" is the axis's
            # dry friction, i.e. what that drivetrain costs before the axis moves
            # at all -- the knob that makes a well-tuned PID stick short of its
            # setpoint.
            for key, text, value, lo, hi in (
                    ("Kp", "Kp", KP[axis], 0.0, KP_MAX[axis]),
                    ("Kd", "Kd", KD[axis], 0.0, KD_MAX[axis]),
                    ("Ki", "Ki", KI[axis], 0.0, KI_MAX[axis]),
                    ("Tmax", f"max torque ({MOTOR_UNITS[axis]})",
                     MOTOR_TORQUE[axis], 0.0, MOTOR_TORQUE_MAX[axis]),
                    # Gear sliders read in display units (mm of radius, or a bare
                    # ratio); apply_to scales them back to SI.
                    ("Gear", f"{GEAR_LABELS[axis]} ({GEAR_UNITS[axis]})",
                     GEAR[axis] * GEAR_SCALE[axis],
                     GEAR_MIN[axis] * GEAR_SCALE[axis],
                     GEAR_MAX[axis] * GEAR_SCALE[axis]),
                    ("Fric", f"friction ({unit})",
                     AXIS_FRICTION[axis], 0.0, AXIS_FRICTION_MAX[axis])):
                var = tk.DoubleVar(value=float(value))
                tk.Scale(frame, from_=float(lo), to=float(hi),
                         resolution=(float(hi) - float(lo)) / 400.0,
                         orient="horizontal", length=200, variable=var,
                         label=text).pack(fill="x")
                self.gain_vars[(axis, key)] = var

        # --- blade, box pool and surface friction (sim only) ---
        self.width_var = None
        self.surface_var = None
        self.mass_vars = {}
        self.box_status = None
        self._box_note = ""
        if robot is not None:
            blade = tk.LabelFrame(self.root, text="Blade", padx=6, pady=4)
            blade.pack(fill="x", padx=8, pady=4)
            self.width_var = tk.DoubleVar(value=float(PADDLE_WIDTH))
            tk.Scale(blade, from_=PADDLE_WIDTH_MIN, to=PADDLE_WIDTH_MAX,
                     resolution=0.001, orient="horizontal", length=420,
                     variable=self.width_var,
                     label="width (m)   — mass and yaw inertia follow it"
                     ).pack(fill="x")

            boxes = tk.LabelFrame(self.root, text="Boxes", padx=6, pady=4)
            boxes.pack(fill="x", padx=8, pady=4)

            insert = tk.Frame(boxes)
            insert.pack(fill="x")
            for kind in BOX_KINDS:
                tk.Button(insert, text=f'+ {BOX_SIDE_IN[kind]:g}"', width=7,
                          command=lambda k=kind: self.add_box(k)
                          ).pack(side="left", padx=2)
            tk.Button(insert, text="Clear", command=self.clear_boxes
                      ).pack(side="left", padx=(12, 2))

            self.box_status = tk.Label(boxes, font=("Consolas", 9), justify="left",
                                       anchor="w")
            self.box_status.pack(fill="x", pady=(2, 0))

            # A mass per size, side by side: what is in a carton decides the push
            # as much as the friction under it does.
            mass_row = tk.Frame(boxes)
            mass_row.pack(fill="x")
            for kind in BOX_KINDS:
                var = tk.DoubleVar(value=float(BOX_MASS[kind]))
                top = BOX_MASS_MAX[kind]
                tk.Scale(mass_row, from_=BOX_MASS_MIN, to=top,
                         resolution=top / 200.0, orient="horizontal", length=135,
                         variable=var, label=f'{BOX_SIDE_IN[kind]:g}" mass (kg)'
                         ).pack(side="left", fill="x", expand=True, padx=2)
                self.mass_vars[kind] = var

            self.surface_var = tk.DoubleVar(value=float(SURFACE_FRICTION))
            tk.Scale(boxes, from_=SURFACE_FRICTION_MIN, to=SURFACE_FRICTION_MAX,
                     resolution=0.01, orient="horizontal", length=420,
                     variable=self.surface_var,
                     label="box / platform friction (mu)").pack(fill="x")

        # --- buttons + readout ---
        row = tk.Frame(self.root)
        row.pack(fill="x", padx=8, pady=(0, 4))
        tk.Button(row, text="Centre", command=self.centre).pack(side="left")
        tk.Button(row, text="Reset integral",
                  command=pid.reset).pack(side="left", padx=6)
        tk.Button(row, text="Reset peak torque",
                  command=self.reset_peak).pack(side="left")

        self.readout = tk.Label(self.root, font=("Consolas", 10), justify="left",
                                anchor="w")
        self.readout.pack(fill="x", padx=10, pady=(0, 8))

    def centre(self):
        """Park the blade at the middle of the platform, square on."""
        for var in self.target_vars:
            var.set(0.0)

    # --- drivetrain ---

    def drivetrain(self):
        """The motor sliders as SI vectors: (max shaft torque N*m, gear).

        The gear entry is a pitch radius in m on the linear axes and a bare
        ratio on yaw -- GEAR_SCALE undoes the units the sliders display in.
        """
        torque = np.array([self.gain_vars[(a, "Tmax")].get()
                           for a in range(N_AXES)])
        gear = np.array([self.gain_vars[(a, "Gear")].get() / GEAR_SCALE[a]
                         for a in range(N_AXES)])
        return torque, gear

    def note_effort(self, effort):
        """Fold one control tick's commanded effort into the peak-torque hold.

        Called from the sim loop rather than from `refresh`, so a spike that
        lands between two redraws still registers. Touches no Tk widget, only
        Tk variables, and runs on the same thread as everything else here.
        """
        _, gear = self.drivetrain()
        np.maximum(self.peak_torque, np.abs(motor_torque(effort, gear)),
                   out=self.peak_torque)

    def reset_peak(self):
        self.peak_torque[:] = 0.0

    # --- box pool ---
    # Tk callbacks run between physics batches, never inside one, so writing box
    # poses straight into the sim from here needs no locking.

    def add_box(self, kind):
        """Insert one box of `kind` onto the platform."""
        if all(self.robot.is_box_active(s) for s in BOX_KIND_SLOTS[kind]):
            self._box_note = f"no {kind} boxes left in the pool"
        elif self.robot.spawn_box(BOX_KIND_SLOTS[kind]) is None:
            self._box_note = f"no room on the platform for a {kind} box"
        else:
            self._box_note = ""

    def clear_boxes(self):
        self.robot.park_all_boxes()
        self._box_note = ""

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
        """Push the current slider values into the PID and into the model.

        The gains and the setpoint belong to the controller; the blade width, the
        box masses and both frictions are properties of the machine, so those go
        straight to the sim. The setters there ignore unchanged values, so calling
        this every tick is cheap.

        The PID's output clamp is not a slider of its own any more: it is what
        the motor and its gearing can put on the axis, so it is derived here from
        the torque and gear sliders.
        """
        pid.set_target([var.get() for var in self.target_vars])
        pid.set_gains(
            kp=[self.gain_vars[(a, "Kp")].get() for a in range(N_AXES)],
            kd=[self.gain_vars[(a, "Kd")].get() for a in range(N_AXES)],
            ki=[self.gain_vars[(a, "Ki")].get() for a in range(N_AXES)],
        )
        pid.set_max_force(axis_effort_limit(*self.drivetrain()))
        if self.robot is None:
            return
        self.robot.set_axis_friction([self.gain_vars[(a, "Fric")].get()
                                      for a in range(N_AXES)])
        self.robot.set_paddle_width(self.width_var.get())
        self.robot.set_surface_friction(self.surface_var.get())
        for kind, var in self.mass_vars.items():
            self.robot.set_box_mass(BOX_KIND_SLOTS[kind], var.get())

    def refresh(self, pos, vel, force):
        """Redraw the readout. Tk pumps its own events -- see run_in_sim."""
        if not self.alive:
            return
        err = self.pid.target - pos
        # Flag saturation: at the limit the loop is open, and no amount of gain
        # tuning explains what the blade does next.
        sat = np.abs(force) >= self.pid.max_force - 1e-6

        # Motor side. `lim` is the torque the axis can actually use: raise the
        # torque slider past what MAX_FORCE allows through the current gearing
        # and it stops climbing, which is the only visible sign that the
        # drivetrain has outgrown the actuator.
        torque_max, gear = self.drivetrain()
        gain = gear_gain(gear)
        self.readout.config(text=(
            _axis_row("pos", pos, AXIS_UNITS) + "\n"
            + _axis_row("err", err, AXIS_UNITS) + "\n"
            + _axis_row("eff", force, EFFORT_UNITS, precision=2)
            + ("   SAT " + " ".join(AXIS_LABELS[a] for a in range(N_AXES) if sat[a])
               if sat.any() else "") + "\n"
            + _axis_row("trq", force / gain, MOTOR_UNITS, precision=3) + "\n"
            + _axis_row("peak", self.peak_torque, MOTOR_UNITS, precision=3) + "\n"
            + _axis_row("lim", np.minimum(torque_max, MAX_FORCE / gain),
                        MOTOR_UNITS, precision=3) + "\n"
            + _axis_row("rpm", motor_rpm(vel, gear), RPM_UNITS, precision=0)))
        if self.box_status is not None:
            counts = "   ".join(
                f'{BOX_SIDE_IN[k]:g}" '
                f"{sum(self.robot.is_box_active(s) for s in BOX_KIND_SLOTS[k])}"
                f"/{BOX_SLOTS[k]}" for k in BOX_KINDS)
            self.box_status.config(
                text=f"on platform   {counts}"
                     + (f"\n{self._box_note}" if self._box_note else ""))


def run_in_sim():
    import mujoco.viewer
    from item_sort_sim import ItemSortSim

    robot = ItemSortSim("item_sort.xml", AXIS_NAMES, BOX_NAMES)
    # Drive physics at SIM_HZ so the control loop lands on exact step counts.
    robot.model.opt.timestep = SIM_DT
    robot.reset_state()

    pid = AxisPID(target=start_target, kp=KP, kd=KD, ki=KI, dt=CTRL_DT,
                  integral_limit=INTEGRAL_LIMIT, max_force=FORCE_LIMIT)
    panel = ControlPanel(pid, robot)   # robot: the panel inserts boxes and
                                       # reshapes/retunes the live model

    print_every = int(round(print_period / SIM_DT)) if print_period else 0

    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
        # Look down on the platform -- the useful view for a planar gantry.
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -60
        viewer.cam.distance = 2.6   # frames the whole 4 ft platform

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
            pos, vel = robot.get_pos(), robot.get_vel()
            for _ in range(max(owed, 0)):
                pos, vel = robot.get_pos(), robot.get_vel()
                if loop["tick"] % SIM_PER_CTRL == 0:
                    loop["force"] = pid.compute(pos, vel)
                    robot.set_force(loop["force"])
                    # Peak-hold here, not in refresh: a spike between two
                    # redraws is exactly the one worth catching.
                    panel.note_effort(loop["force"])
                robot.step_n(1)   # one physics step at SIM_HZ
                loop["tick"] += 1
                if print_every and loop["tick"] % print_every == 0:
                    print(f"t={robot.data.time - sim_start:6.2f}s  "
                          f"{_axis_row('pos', pos, AXIS_UNITS)}  "
                          f"{_axis_row('eff', loop['force'], EFFORT_UNITS)}")

            viewer.sync()
            panel.refresh(pos, vel, loop["force"])
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
    drive backend, home all three axes against their end stops to establish the
    zero that item_sort_config.TRAVEL is measured from, then run
    pid.compute(pos, vel) at CTRL_HZ, writing the resulting force (or torque, on
    yaw, or their current-loop equivalents) to each axis. Ensure a safe stop --
    zero effort, brakes engaged -- on exit.
    """
    raise NotImplementedError("item_sort hardware is not implemented yet")


if __name__ == "__main__":
    try:
        run_in_sim()
    except KeyboardInterrupt:
        pass

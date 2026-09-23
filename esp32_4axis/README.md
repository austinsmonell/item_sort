# esp32_4axis — 4-axis stepper controller

ESP32-WROOM-32U driving three HB808C closed-loop hybrid servo drives and one
open-loop stepper. Controlled over USB serial, WiFi, or a browser page; reflashed
over the air.

Two files, and they are a matched pair — the GUI reads config keys and status
fields the firmware emits, so flash and reload together:

| File | What it is |
|---|---|
| `esp32_4axis.ino` | The firmware. Single Arduino sketch. |
| `stepper_gui.html` | The control panel — a **CONTROL** tab and a **GANTRY VIEW** tab (top-down work-area, click to move). See [`GUI_GUIDE.md`](GUI_GUIDE.md). |

---

## Hardware

### Pin map

| Axis | PUL+ | DIR+ | ENA+ | Encoder A / B | Drive |
|---|---|---|---|---|---|
| 1 | 25 | 26 | 23 | 36 / 39 | HB808C, gantry end A |
| 2 | 27 | 14 | 22 | 34 / 35 | HB808C, gantry end B |
| 3 | 18 | 5 | 21 | 32 / 33 | HB808C |
| 4 | 17 | 16 | 19 | — | open loop |

Pins deliberately avoided or chosen:

- **GPIO 12 is unused.** It sets flash voltage at boot.
- **GPIO 5 and 14 carry DIR, never PUL.** GPIO 14 emits JTAG activity during
  boot, which on a pulse line would be real steps.
- **GPIO 36/39/34/35 are input-only** with no internal pulls. The encoders drive
  both states actively, so none are needed.
- **ENA on 23/22/21/19** — none is a strapping pin, so all four are safe to drive
  from reset.

### Wiring — common cathode

All `PUL-` / `DIR-` / `ENA-` tie to ground, shared with the ESP32. The GPIO drives
the **plus** lines directly. Logic is therefore normal: **GPIO HIGH = optocoupler
conducting = signal asserted.**

This is the right way round, and the reason matters. An earlier revision tied the
plus lines to 5 V and sank the minus lines. A 3.3 V GPIO high then left 1.7 V
across the opto — above the LED forward voltage — so a couple of mA kept flowing
and a de-asserted input never actually cleared. Edge-triggered PUL survived that;
a level-sensitive DIR did not, which is what pinned an axis to one direction.
Common cathode gives a true 0 V off state.

The cost is drive current: 3.3 V into an input resistor sized for 5 V is roughly
8 mA where the driver expected 14 mA. That is inside spec for the usual opto
inputs. **If one driver is marginal, lower its series resistor — do not go back
to a 5 V anode.**

Encoder `EA+`/`EB+` are tapped through a TXS0108E level shifter (5 V → 3.3 V).

`ALM` is not wired. See [Faults you cannot see](#faults-you-cannot-see).

### ENA polarity

The firmware assumes **a conducting ENA input disables the drive** — which is why
leaving ENA unwired is the running state on these drivers. The pins idle LOW at
reset, so a board that reboots comes back with the drives live, not dead.

If a drive reads its ENA input the other way, the symptom is unmistakable (that
axis comes up dead, or the DRIVES button works backwards). Fix it per axis with
`C <axis> eni 1` — no rewiring.

---

## Building and flashing

Libraries, all from the Arduino Library Manager:

- **FastAccelStepper** by gin66
- **ESP32Encoder** by Kevin Harrington
- **WebSockets** by Markus Sattler (`arduinoWebSockets`)

Board: ESP32 Dev Module. Serial monitor at **115200**, line ending **Newline**.

Edit `WIFI_SSID` and `WIFI_PASS` near the top before the first flash. After that
the board appears in *Tools → Port* as a network port; `OTA_PASS` unlocks it.

> **FLASH WITH THE MOTOR SUPPLY OFF.** Every upload resets the board and leaves
> the unbuffered PUL lines floating, which can emit stray steps.

> **The WROOM-32U has no antenna on it.** The `-U` module brings RF out to a U.FL
> connector instead of a PCB trace. Without a pigtail fitted it will associate
> from a few inches away and nowhere else.

### Config resets on any struct change

Config is persisted per axis as a raw `AxisConfig` blob in NVS. `loadConfig()`
checks the stored length against `sizeof(AxisConfig)` and **ignores a blob that
does not match**, rather than part-filling the struct and leaving new fields as
junk. So any change to `AxisConfig` — adding a field, reordering — silently
reverts every axis to the compiled defaults on the next boot.

Take a backup from the GUI (`SAVE TO FILE`) before flashing a build that changes
the struct. Restoring skips keys the file does not contain, so an older backup is
still useful.

The machine-wide idle timeout lives on its own NVS key (`idle`) and survives
struct changes.

---

## The axis model

**Axes 1 and 2 are two motors on opposite ends of one rail.** Commanding one
alone racks it, so nothing may. Jog, move-to, zero and home all fan out to both
ends; the motion config keys mirror across the pair; a fault on one end stops the
other. `commandMove()`, `commandZero()` and `commandHome()` are the only entry
points — no caller touches an axis index directly.

Check that the two ends move the same way before homing. If they fight, flip one
with `C 2 inv 1` rather than rewiring.

Axis 3 is a single closed-loop drive. Axis 4 is open loop with no encoder, which
limits what the firmware can know about it — it cannot detect a stall, cannot
verify position after a drive disable, and homes only by dead reckoning.

---

## Homing

There is no home switch. "I reached the hard stop" is inferred from the encoder
refusing to follow the step generator.

### The sequence

```
approach  →  touch  →  backoff  →  precise  →  touch2  →  offset  →  homed
(fast)       (wait)                (slow)      (wait)
```

1. **approach** — drive at `hs` until stall.
2. **touch** — stop and hold at the stop. *Wait for the gantry partner.*
3. **backoff** — both ends retreat `bo` steps together.
4. **precise** — creep back in at `hss`, bounded to `bo × 3`.
5. **touch2** — stop, hold, and call this position zero. *Wait for the partner.*
6. **offset** — both ends move `wo` steps off the stop together.
7. **homed** — that position becomes zero, and `isHomed` is set.

The slow second touch is the repeatable one, which is why zero comes from there
and not from the fast approach. Each end zeros on **its own** touch — that
per-end reference is what squares the rail. Only the moves off the stop are
shared.

### Why the two waits exist

Each end used to run its FSM independently, and the header comment assumed the
touches were simultaneous. They are not: whichever end starts closer touches
first, backs off, and drives *away* from the stop while its partner is still
driving *toward* it. Two closed-loop motors then fight across a rigid rail, both
ramping to peak current, until a drive latches its internal position-error alarm.
With ALM and ENA unwired at the time, that needed a power cycle to clear.

`H_TOUCH_1` and `H_TOUCH_2` are the fix. No end ever leaves a stop on its own.

### Stall detection is windowed

`stallDetected()` measures following error over a **sliding window of commanded
steps** (`STALL_WINDOW_STEPS`, 100), never over the whole approach.

Integrating from the start of the move made the effective margin shrink with
distance — 40 counts is 1% slip over 2000 steps but 0.02% over 100 000 — so a
long approach would trip a phantom stall in mid-air, and on the gantry that meant
the fight above. Re-baselining every window makes detection independent of how
far out homing started.

Sizing: a real stall adds `countsPerStep` of error per commanded step, so it
clears `thr` in `thr / countsPerStep` steps — 20 with the stock numbers. The
first window of each approach (`STALL_BLANK_STEPS`, 50) is unarmed, because it
spans the acceleration ramp where the encoder legitimately lags.

### Axis 4 homes blind

`homeEnable` is 0 for axis 4 and should stay that way. With no encoder, the only
way it can find a stop is to drive into it and keep pushing — a stalled motor at
full current for whatever is left in `mt`. Nothing can detect arrival. Fit a
limit switch, or zero it by hand with `Z 4`.

If you ever enable it, **fix `mt` first.** It is 64000 steps, sized when the axis
was assumed to be 200 steps/mm. The measured calibration is 167.9 steps/mm, so
64000 steps is 381 mm on a 280 mm axis.

---

## Travel calibration and soft limits

Nothing in the config derives linear travel — `ppr` says nothing about pulley
diameter or screw lead. `spm` (steps per metre) is **measured**: drive a known
distance and put a rule on it.

| Axis | Measured | `spm` |
|---|---|---|
| 1 + 2 | 11000 steps = 100.8 cm | 10913 |
| 3 | 16000 steps = 148.5 cm | 10774 |
| 4 | 47000 steps = 28.0 cm | 167857 |

Steps per **metre** rather than per mm so an integer keeps the precision: 10913
is 0.003% off, which is 0.03 mm over a whole axis.

`lmin` / `lmax` are soft travel limits in **whole millimetres** (the GUI shows
them as cm). `lmax <= lmin` turns the limit off for that axis.

Four things about how the clamp behaves:

- **Always enforced, never absent.** A limit is measured from a zero, and the
  axis always has one: the power-on zero, a `Z`, or a completed home. Only the
  last two are surveyed against a hard stop, so before the axis is homed or
  zeroed the window is *approximate* — measured from wherever the board booted —
  but it is still applied. That is what keeps a jog or an `M` from running an
  un-homed axis, or one that lost `isHomed` after a fault, clean off the end.
  The status line's per-axis `lim` flag says whether a limit is configured;
  `state` (idle vs homed) says whether the window is surveyed yet.
- **It cannot trap an un-homed axis.** Homing drives the stepper directly and
  never goes through the clamp, so `H` always reaches the stop. To reach a spot
  outside the power-on window by hand first, `Z` the axis there.
- **The firmware owns the target.** `cmdTarget[]` holds where the last accepted
  command aims each axis, and every move goes out as an absolute `moveTo()`.
  `getPositionAfterCommandsCompleted()` cannot be used for this: it reports the
  end of the step *queue*, a few milliseconds of steps, not the end of the ramp,
  so a burst of jogs each measured its delta from wherever the axis happened to
  be and stacked straight through the limit. Nor can `getCurrentPosition()` be
  re-read on every jog: a jog can land before FastAccelStepper flips
  `isRunning()` true, and re-syncing there throws away the accumulated target on
  every keystroke, so a held or spammed jog key walks past the limit one
  un-counted press at a time. `cmdTarget[]` is reconciled with the hardware
  only once the axis has settled (`MOVE_SETTLE_MS`); anything that moves an axis
  outside `issueMove()` — homing, `S`, a fault, `Z` — resets it explicitly.
- **Relative stays relative.** A clamped jog sends the same *delta* to both
  gantry ends. Turning it into an absolute target taken from one end would snap
  the other across the skew and rack the rail.

Homing bypasses the limits entirely — it drives the stepper directly. The `T`
self-test is clamped like any other move and skips its step if there is no room.

---

## Drive enable and the idle timeout

`EN 1` / `EN 0` energises and releases all drives. `IDLE <seconds>` releases them
after that long with nothing moving (default 300, `0` = never).

Cutting holding torque has two consequences, both handled:

- **A moving axis is never dropped.** `setDrives(false)` stops everything first,
  or the load carries on under its own momentum with nothing resisting.
- **A disabled axis may not be where it was left.** The encoders keep counting
  whether the drive is powered or not, so on re-enable each closed-loop axis is
  asked whether it actually moved; those that drifted past `thr` lose `isHomed`
  with a message naming the count. Axis 4 has no encoder, cannot be asked, and so
  always loses homing.

Jog, move-to, home and `T` all **refuse** while the drives are off rather than
auto-enabling. Re-enabling is where the drift check reports, and doing it
silently under a jog would bury the one message worth reading.

> **`FD` — fault-drop.** On by default: any axis fault releases every drive, the
> same as `E 1`. `FD 0` keeps the drives energised on a fault (motion still stops
> and the axis still latches `FAULT`). Persisted across reboots. After a
> fault-drop, investigate, then `EN 1` and re-home the latched axes. The status
> line carries it as `fdrop`.

Homing counts as activity for the timeout, including the `touch` states where
both gantry ends stand still waiting for each other. A clock must not cut the
current out from under a half-finished home.

> **`ihold`.** Set `C <axis> ihold 1` and that axis rides out the idle timeout
> still energised. Releasing a drive that is holding a load against gravity drops
> it. `ihold` only holds off the *clock* — `EN 0` and `E 1` are deliberate
> instructions and release everything. Axis 4's T8×8 screw has a ~17° lead angle,
> right on the edge of back-driving; if it is mounted vertically, set `ihold 1`
> on it.

---

## Safety

**WiFi is not an e-stop path.** A dropped link cannot stop a moving machine any
faster than its timeout. The board watches for that itself: if the last WebSocket
client goes away while an axis is moving, motion stops. That is a backstop, not a
safety device. **Keep a hardware e-stop in the motor supply.**

`E 1` engages a software e-stop: it stops anything running, **releases every
drive** (the same as `EN 0`, and including an `ihold` axis — a software e-stop
goes as dark as a hardware one), and blocks all motion commands. `E 0` releases
the e-stop but leaves the drives off; bringing them back needs an explicit
`EN 1`. It is still a software path on a shared MCU — **keep a hardware e-stop in
the motor supply.**

### Faults you cannot see

`ALM` is unwired, so a latched drive alarm is invisible to the firmware.
`serviceSyncWatch()` is the only way it gets noticed: while an axis is running
outside homing, commanded steps are compared against encoder counts every 100 ms,
and error past `thr × 3` faults the axis with `lost sync - check drive LED`.

There is no such watchdog during homing. If a drive latches mid-approach, the
motor stops moving while pulses keep going out — which `stallDetected()` reads as
"arrived at the hard stop." It will call that position zero and report a
successful home. **Wiring ALM would turn that into a real fault.** Wiring nothing
means trusting the drive LEDs.

A gantry end that faults takes its partner down with it. One end still driving
while the other has stopped is exactly how a rail racks.

Any fault also releases every drive unless `FD 0` has turned that off — see
**Drive enable and the idle timeout** above.

---

## Command reference

Every command is one line, identical on USB serial and over the WebSocket. Axes
are numbered **1–4**; axis 1 and 2 are one gantry and always move together.

| Command | Effect |
|---|---|
| `J <axis> <steps>` | Jog. Negative reverses. Adds to a move already in flight. |
| `M <axis> <pos>` | Move to an absolute step position. |
| `H <axis>` | Home one axis. `H 1` or `H 2` homes both gantry ends. |
| `HA` | Home every enabled axis, one at a time. |
| `Z <axis>` | Set the current position as zero. |
| `S` | Stop all motion. |
| `EN 1` / `EN 0` | Energise / release all drives. `EN` alone reports. |
| `IDLE <sec>` | Idle release timeout, `0` = never. `IDLE` alone reports. |
| `FD 1` / `FD 0` | Fault-drop: release all drives on any fault (default on, persisted). `FD` alone reports. |
| `E 1` / `E 0` | Engage / release the software e-stop. `E 1` also cuts every drive; `EN 1` to re-energise after `E 0`. |
| `C <axis> <key> <val>` | Set a config key, saved to flash immediately. |
| `?` | Print one status line. |
| `V 1` / `V 0` | Status streaming on / off (default on, every 250 ms). |
| `T <axis>` | Self test: report state, then step slowly. |
| `IP` | WiFi address, signal strength, WebSocket client count. |
| `HELP` | The list above. |

`T` answers the only question that matters when an axis is dead: is the board
failing to command it, or is it commanding fine and nothing downstream is
listening?

---

## Config keys

`C <axis> <key> <value>`. Every write saves to flash immediately.

Keys marked **shared** mirror across gantry axes 1 and 2 — set either end and
both take it. The rest describe one motor and stay per axis.

| Key | Unit | Default | Shared | Meaning |
|---|---|---|---|---|
| `run` | steps/s | 2000 | ● | Jog and move-to speed. |
| `acc` | steps/s² | 8000 | ● | Acceleration, all moves. |
| `hs` | steps/s | 600 | ● | First (fast) homing approach. |
| `hss` | steps/s | 120 | ● | Second (precise) homing approach. |
| `ppr` | pulses/rev | 2000 / 1600 | | Must match the driver DIP switches. |
| `cpr` | counts/rev | 4000 | | Encoder line count × 4. `0` = no encoder. |
| `thr` | counts | 40 | | Following error that counts as a stall. |
| `bo` | steps | 400 | ● | Retreat between homing approaches. |
| `wo` | steps | 800 | ● | Distance off the stop that becomes zero. |
| `mt` | steps | 200000 | ● | Give up homing after this much travel. |
| `dir` | ±1 | −1 | ● | Which way the hard stop lies. |
| `inv` | 0/1 | 0 | | Flip this motor's sense (mirrored mount). |
| `he` | 0/1 | 1 | ● | `0` refuses to home this axis at all. |
| `spm` | steps/m | measured | ● | Linear calibration. See above. |
| `lmin` | mm | 0 | ● | Soft lower travel limit. |
| `lmax` | mm | measured | ● | Soft upper travel limit. `<= lmin` = off. |
| `eni` | 0/1 | 0 | ● | `1` if a conducting ENA input *enables* the drive. |
| `ihold` | 0/1 | 0 | ● | `1` keeps the axis live through the idle timeout. |

### Tuning `thr`

`thr` is a following error in encoder counts. At `cpr 4000` / `ppr 2000` there are
2.0 counts per step, so `thr 40` is 20 steps of lost motion — 1% of a revolution.

Too low and belt stretch, backlash and coupling wind-up trip it. Too high and the
axis keeps pushing after it has arrived, which is a stalled motor at full current
and makes the touch position depend on how hard it drove in — the opposite of the
repeatability the slow approach exists to get.

Watch `err` in the status line while jogging to pick a value with real margin.

---

## Status line

One JSON object per line, prefixed with `{` so a client can tell it apart from
the human-readable `#` messages. Emitted every 250 ms while streaming is on, or
once on `?`.

```json
{
  "estop": false,
  "drives": true,
  "idle": 300,
  "fdrop": true,          // fault-drop safety switch (FD)
  "axis": [
    {
      "pos": 5400,          // commanded step position
      "enc": 10800,         // raw encoder counts
      "err": 0.0,           // following error, counts
      "moving": false,
      "state": "homed",     // see below
      "lim": true,          // is a soft limit configured for this axis
      "c": { "run": 2000, "acc": 8000, "...": "every config key" }
    }
  ]
}
```

`state` is one of `idle`, `approach`, `touch`, `backoff`, `precise`, `touch2`,
`offset`, `homed`, `FAULT`, `blind`.

`err` means two different things depending on context, both in encoder counts:
during a homing approach it is the error accumulated in the current detection
window; while running normally it is the error over the last 100 ms sample. It is
latched on fault — frozen at the value that tripped it — which is deliberate, as
that is the forensic evidence.

---

## Troubleshooting

| Symptom | Look at |
|---|---|
| An axis does nothing | `T <axis>`. Reports whether the stepper attached, the e-stop and drive state, and then steps slowly. If it steps and the motor does not move, the problem is downstream. |
| An axis is stuck in one direction | DIR wiring. This is the exact symptom of the old 5 V-anode wiring — check the plus lines are GPIO-driven and the minus lines grounded. |
| `lost sync - check drive LED` | A latched drive alarm, most likely. ALM is unwired so this watchdog is the only notice you get. |
| Homing faults with `no stop in N steps` | The axis ran the whole of `mt` without stalling. Either `dir` is backwards or `thr` is too high. |
| Homing faults with `lost contact on slow approach` | The precise approach ran `bo × 3` steps without reaching the stop. Usually `bo` set too large. |
| A drive latches during homing | Both ends should now be gated by `touch`/`touch2`. If it still happens, watch the state chips — an end sitting in `touch` for more than a second or two means the ends are arriving far apart. |
| Limits are ignored | Check `lim` in the status line, or the travel row in the GUI — `lmax <= lmin` turns the limit off. Limits are enforced on an un-homed axis too, but from the power-on zero, so the window can sit in the wrong place until you home or `Z`. |
| Settings reverted after a flash | Expected if `AxisConfig` changed size. Restore from a GUI backup file. |
| Board associates only from inches away | No U.FL antenna fitted on the WROOM-32U. |

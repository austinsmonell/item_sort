// =====================================================================
// Soft travel limits, and the jog / move-to / zero / home command paths.
// These are the entry points for motion: each one fans the command out to
// both ends of the gantry, so no caller - GUI or serial monitor - can move
// one end on its own.
// =====================================================================
static inline long mmToSteps(int a, int32_t mm) {
  return (long)(((int64_t)mm * (int64_t)cfg[a].stepsPerM) / 1000);
}

// A span of zero or less switches the limit off for that axis.
static inline bool limitsActive(int a) {
  return cfg[a].limitMaxMm > cfg[a].limitMinMm;
}

// A soft limit is measured from a zero, and this axis has one of three: the
// power-on zero (setCurrentPosition(0) in setup), a Z, or a completed home.
// Only the last two are surveyed against a hard stop - after power-on, zero is
// just wherever the board booted. An un-surveyed window is still worth
// enforcing though: it keeps a jog or an M from running the axis clean off the
// end, which is exactly what used to happen before the axis was homed, or
// after a fault / sync-loss / disable cleared isHomed. So the window is
// APPROXIMATE until the axis is homed or zeroed - never absent.
//
// This cannot trap an un-homed axis. Homing drives the stepper directly and
// never comes through here, so H always reaches the stop. To reach a spot
// outside the power-on window by hand first, Z the axis there to set a fresh
// zero.
static long clampToLimits(int a, long target) {
  if (!limitsActive(a)) return target;
  long lo = mmToSteps(a, cfg[a].limitMinMm);
  long hi = mmToSteps(a, cfg[a].limitMaxMm);
  if (target < lo) return lo;
  if (target > hi) return hi;
  return target;
}

// cmdTarget[] / cmdTargetMs[] are declared in 01_config.ino with the other
// globals. The library cannot stand in for cmdTarget: getPositionAfterCommandsCompleted()
// reports the end of the step QUEUE, a few milliseconds of steps, not the end
// of the ramp, so a burst of jogs read off it each measured its delta from
// wherever the axis happened to be and stacked straight through the limit.
//
// getCurrentPosition() has the opposite failure during a burst. A jog can
// arrive before FastAccelStepper has flipped isRunning() true (the same lag
// MOVE_SETTLE_MS covers in homing), and in that gap the axis still reads its
// old position. Re-syncing cmdTarget from it there throws the accumulated
// target away on every keystroke, so a held jog key walks the axis one
// un-counted press at a time - past the soft limit if it is sitting on one.
// So reconcile with the hardware ONLY once the axis has actually settled;
// until then cmdTarget is the authority and the limit is tested against it.
static void syncTargetIfSettled(int a) {
  if (!stepper[a]->isRunning() && millis() - cmdTargetMs[a] > MOVE_SETTLE_MS)
    cmdTarget[a] = stepper[a]->getCurrentPosition();
}

// Everything goes out as an absolute moveTo against a target this file owns,
// so no relative-move bookkeeping inside the library can be raced.
static void issueMove(int a, long v, bool absolute) {
  if (!stepper[a]) return;
  syncTargetIfSettled(a);

  long target = clampToLimits(a, absolute ? v : cmdTarget[a] + v);
  applyAxisSpeed(a, cfg[a].runSpeed);
  stepper[a]->moveTo(target);
  setCmdTarget(a, target);
}

// Clamped once, against the commanding axis, and the SAME adjustment goes to
// both ends of a pair. A relative move stays relative: the two ends sit at
// their own stop-referenced zeros, so turning a jog into an absolute target
// taken from one end would snap the other across the skew and rack the rail.
//
// Position after commands completed, not current position: J adds to the move
// already in flight, so that is the number the limit has to be tested against
// or stacked jogs would walk straight through it.
static void commandMove(int a, long v, bool absolute) {
  if (!stepper[a]) return;
  syncTargetIfSettled(a);

  long want    = absolute ? v : cmdTarget[a] + v;
  long clamped = clampToLimits(a, want);

  if (clamped != want)
    outf("# axis %d limited to %ld steps (%s window, travel %ld to %ld mm)",
         a + 1, clamped, st[a].isHomed ? "homed" : "power-on",
         (long)cfg[a].limitMinMm, (long)cfg[a].limitMaxMm);

  if (absolute) {
    issueMove(a, clamped, true);
    if (isGantry(a)) issueMove(partnerOf(a), clamped, true);
  } else {
    // Already sitting on the limit: send nothing rather than a zero-length
    // move, so holding the jog key cannot keep restarting the ramp.
    long dv = clamped - cmdTarget[a];
    if (dv == 0) return;
    issueMove(a, dv, false);
    if (isGantry(a)) issueMove(partnerOf(a), dv, false);
  }
}

static void zeroOne(int a) {
  if (!stepper[a]) return;
  stepper[a]->setCurrentPosition(0);
  if (hasEncoder(a)) encoder[a].setCount(0);
  setCmdTarget(a, 0);
  st[a].isHomed  = true;
  st[a].home     = H_IDLE;
  st[a].fault[0] = '\0';
  outf("# axis %d zeroed", a + 1);
}

// Zeroing both ends where they stand keeps whatever squareness they have.
static void commandZero(int a) {
  zeroOne(a);
  if (isGantry(a)) zeroOne(partnerOf(a));
}

// Both ends run their own state machine into their own hard stop at the
// same time: that simultaneous touch is what squares the rail.
static void commandHome(int a) {
  if (!cfg[a].homeEnable) {
    outf("# axis %d homing disabled - Z %d to zero it here, or C %d he 1",
         a + 1, a + 1, a + 1);
    return;
  }
  beginHome(a);
  if (isGantry(a)) beginHome(partnerOf(a));
}

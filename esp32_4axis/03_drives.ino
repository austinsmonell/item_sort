// =====================================================================
// Drive control: direction/enable pins, speed, enable/disable, faults,
// and the idle timeout that eventually calls setDrives(false, ...).
// =====================================================================

// DIR must be settled before the first pulse of a move. FastAccelStepper
// defaults to no setup time, which the HB808C tolerates but a slower opto
// does not: the first steps of a reversal then go out under the old
// direction. Note the library clamps this to MIN_DIR_DELAY_US, which is
// 200 us on esp32 - asking for less does not get you less.
#define DIR_SETUP_US 200

// Common cathode: DIR- is grounded, so the GPIO must SOURCE current to
// light the opto and a high asserts. That makes the pins push-pull, which
// is not a free choice - an open-drain output can only sink, so it could
// never assert a common-cathode input at all.
//
// dirHighCountsUp is therefore true in the normal case, the opposite of
// the old common-anode wiring. invertDir still flips one motor of a pair
// that is mounted mirrored to its partner.
static void applyDirection(int a) {
  if (!stepper[a]) return;
  stepper[a]->setDirectionPin(DIR_PIN[a], cfg[a].invertDir == 0, DIR_SETUP_US);
}

// Common cathode, same as PUL and DIR: driving ENA+ HIGH lights the opto.
// A conducting ENA input is what switches these drives OFF, so "enabled"
// is the dark, LOW state - which is exactly what the pins do at reset.
static void applyEnable(int a) {
  bool level = !axisLive[a];                    // HIGH asserts ENA = disabled
  if (cfg[a].enaInvert) level = !level;
  digitalWrite(ENA_PIN[a], level ? HIGH : LOW);
}

static void applyAxisSpeed(int a, uint32_t speed) {
  if (!stepper[a]) return;
  stepper[a]->setSpeedInHz(speed);
  stepper[a]->setAcceleration(cfg[a].accel);
}

static void stopAll() {
  homeChain = -1;
  for (int a = 0; a < NUM_AXES; a++) {
    if (stepper[a]) {
      stepper[a]->forceStopAndNewPosition(stepper[a]->getCurrentPosition());
      setCmdTarget(a, stepper[a]->getCurrentPosition());   // shadow follows the abrupt stop
    }
    if (st[a].home != H_FAULT && st[a].home != H_DONE) st[a].home = H_IDLE;
  }
}

// Cutting the current to a stepper releases its holding torque: the axis can
// then be pushed, or fall. Two consequences are handled here.
//
// Never drop a moving axis - stop it first, or the load carries on under its
// own momentum with nothing resisting.
//
// And a disabled axis may not be where it was left. The encoders keep
// counting whether the drive is powered or not, so on re-enable a closed loop
// axis can be asked whether it actually moved, and only the ones that did
// lose their homing. Axis 4 has no encoder and cannot be asked, so it always
// does - there is no honest alternative.
//
// fromIdle separates the clock from the operator. An axis with idleHold set
// rides out the timeout still energised - a drive holding a load against
// gravity must not be released because nothing happened for a while - but
// EN 0 is a deliberate instruction and releases everything.
static void setDrives(bool on, bool fromIdle) {
  if (on == drivesEnabled) return;

  if (!on) {
    stopAll();
    for (int a = 0; a < NUM_AXES; a++) enaCounts[a] = encCount(a);
  }

  drivesEnabled = on;
  int held = 0;
  for (int a = 0; a < NUM_AXES; a++) {
    axisLive[a] = on || (fromIdle && cfg[a].idleHold);
    if (!on && axisLive[a]) held++;
    applyEnable(a);
  }
  lastMotionMs = millis();

  if (on) {
    for (int a = 0; a < NUM_AXES; a++) {
      if (!st[a].isHomed) continue;
      if (!hasEncoder(a)) {
        st[a].isHomed = false;
        outf("# axis %d: open loop, position unverifiable after a disable - re-home", a + 1);
      } else {
        int64_t drift = llabs(encCount(a) - enaCounts[a]);
        if (drift > (int64_t)cfg[a].stallThreshold) {
          st[a].isHomed = false;
          outf("# axis %d moved %ld counts while disabled - re-home", a + 1, (long)drift);
        }
      }
    }
  }

  if (held) outf("# drives DISABLED (%d held live by ihold)", held);
  else      outf("# drives %s", on ? "ENABLED" : "DISABLED");
}

// Release every drive and halt everything, the way E 1 does. setDrives() no-ops
// when the drives are already logically off, so the hand sweep afterwards is
// what guarantees an idleHold axis goes dark too. drivesEnabled is left off:
// nothing re-energises without a deliberate EN 1.
static void killDrives() {
  stopAll();
  setDrives(false, false);
  for (int a = 0; a < NUM_AXES; a++) {
    if (axisLive[a]) enaCounts[a] = encCount(a);
    axisLive[a] = false;
    applyEnable(a);
  }
}

// A gantry end that faults has to take its partner down with it: one end
// still driving while the other has stopped is exactly how a rail racks.
static void faultAxis(int a, const char* msg) {
  if (st[a].home == H_FAULT) return;
  if (stepper[a]) {
    stepper[a]->forceStopAndNewPosition(stepper[a]->getCurrentPosition());
    setCmdTarget(a, stepper[a]->getCurrentPosition());
  }
  st[a].home    = H_FAULT;
  st[a].isHomed = false;
  snprintf(st[a].fault, sizeof(st[a].fault), "%s", msg);
  outf("# axis %d FAULT: %s", a + 1, msg);
  if (isGantry(a)) faultAxis(partnerOf(a), "partner faulted");   // recurses once

  // Safety switch (FD, on by default): a fault drops every drive. The
  // drivesEnabled guard means the gantry recursion above does not fire this
  // twice - the first call takes the drives down, the second sees them gone.
  if (faultKillsDrives && drivesEnabled) {
    outLine("# fault - all drives OFF (FD 0 to disable this)");
    killDrives();
  }
}

static bool anyMoving() {
  for (int a = 0; a < NUM_AXES; a++)
    if (stepper[a] && stepper[a]->isRunning()) return true;
  return false;
}

// True while axis a (or its gantry partner) is mid-homing - not idle, not
// done, not faulted. Used both to keep the idle timeout from cutting power
// mid-home and to block a jog/move while homing owns the axis.
static bool busyHoming(int a) {
  HomeState h = st[a].home;
  if (h != H_IDLE && h != H_DONE && h != H_FAULT) return true;
  if (isGantry(a)) {
    HomeState p = st[partnerOf(a)].home;
    if (p != H_IDLE && p != H_DONE && p != H_FAULT) return true;
  }
  return false;
}

// Idle timeout. Homing counts as activity even in the touch states, where
// both ends are deliberately standing still waiting for each other - a clock
// must not cut the current out from under a half-finished home.
static void serviceIdle() {
  bool active = anyMoving();
  for (int a = 0; a < NUM_AXES && !active; a++) if (busyHoming(a)) active = true;

  if (active) { lastMotionMs = millis(); return; }
  if (!drivesEnabled || idleTimeoutS == 0) return;
  if (millis() - lastMotionMs < idleTimeoutS * 1000UL) return;

  outf("# idle %u s - dropping the drives", idleTimeoutS);
  setDrives(false, true);
}

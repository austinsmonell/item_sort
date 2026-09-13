// =====================================================================
// Homing state machine, plus the running "lost sync" watch that uses the
// same following-error math once an axis is idle/homed.
// =====================================================================

// Arduino's auto-prototype scanner only reads the raw .ino tabs, never the
// headers they #include, so the copies of these in config.h are invisible
// to it and it still tries to generate its own - landing before HomeState
// is declared and failing to build. A hand-written prototype IN A TAB is
// what it actually checks for before generating one, so it goes here too.
static bool pairAt(int a, HomeState h);
static void holdAtTouch(int a, HomeState touch);

// Stall detection runs over a sliding window of commanded steps, never
// over the whole approach. Integrating from the start of the move made the
// effective margin shrink with distance - 40 counts is 1% slip over 2000
// steps but 0.02% over 100000 - so a long approach would trip a phantom
// stall in mid-air. On the gantry that is worse than a false alarm: the
// end that trips backs off while its partner is still driving in, and the
// two motors fight across the rail until a driver latches its position
// error alarm, which with ALM and ENA unwired needs a power cycle to
// clear. Re-baselining every window makes detection independent of how far
// out homing started.
//
// Sizing: a real stall adds countsPerStep of error per commanded step, so
// it clears stallThreshold in thr/countsPerStep steps - 20 with the stock
// numbers. A window several times that leaves the trip comfortable while
// giving slow drift no room to accumulate.
static const int32_t STALL_WINDOW_STEPS = 100;

// The first window of an approach spans the acceleration ramp, where the
// encoder legitimately lags. Blank that one, run armed from there on.
static const int32_t STALL_BLANK_STEPS = 50;

static void armStallWindow(int a) {
  if (!stepper[a]) return;
  st[a].winSteps  = stepper[a]->getCurrentPosition();
  st[a].winCounts = encCount(a);
  st[a].winArmed  = false;
  st[a].lastError = 0.0f;
}

// FastAccelStepper does not always report isRunning() on the same pass a
// move is issued, and both ends of a pair are now released inside a single
// pass, so "stopped" has to mean "stopped, and the command has had time to
// take". This also covers a zero-length move - backoffSteps or workOffset
// set to 0 - which never runs at all and would otherwise hang the state
// machine waiting for a motion that never starts.
static const uint32_t MOVE_SETTLE_MS = 5;

static bool moveFinished(int a) {
  if (millis() - st[a].moveMs < MOVE_SETTLE_MS) return false;
  return !stepper[a]->isRunning();
}

static void beginHome(int a) {
  if (estopActive || !stepper[a]) return;

  st[a].isHomed = false;
  st[a].fault[0] = '\0';
  armStallWindow(a);

  if (!hasEncoder(a)) {
    st[a].home = H_DEADRECKON;
    applyAxisSpeed(a, cfg[a].homeSpeedSlow);
    stepper[a]->move(cfg[a].homeDir * (int32_t)cfg[a].maxHomeTravel);
    st[a].moveMs = millis();
    return;
  }

  st[a].home = H_APPROACH_FAST;
  applyAxisSpeed(a, cfg[a].homeSpeed);
  stepper[a]->move(cfg[a].homeDir * (int32_t)cfg[a].maxHomeTravel);
  st[a].moveMs = millis();
}

static bool stallDetected(int a) {
  if (!hasEncoder(a) || !stepper[a]) return false;

  int32_t stepsMoved = abs(stepper[a]->getCurrentPosition() - st[a].winSteps);
  int64_t encMoved   = llabs(encCount(a) - st[a].winCounts);
  st[a].lastError    = (float)stepsMoved * countsPerStep[a] - (float)encMoved;

  if (st[a].winArmed && st[a].lastError > (float)cfg[a].stallThreshold) return true;

  // Window closed with no stall: drop the baseline here and start over, so
  // nothing carries forward into the next one.
  if (stepsMoved >= (st[a].winArmed ? STALL_WINDOW_STEPS : STALL_BLANK_STEPS)) {
    st[a].winSteps  = stepper[a]->getCurrentPosition();
    st[a].winCounts = encCount(a);
    st[a].winArmed  = true;
    st[a].lastError = 0.0f;
  }
  return false;
}

// Both ends of a pair sitting at the same touch state - the condition for
// either of them to move again. A lone axis only answers for itself.
static bool pairAt(int a, HomeState h) {
  if (st[a].home != h) return false;
  return !isGantry(a) || st[partnerOf(a)].home == h;
}

// Park at the stop the instant it is found: stop pulsing and hold, no
// backoff and no zeroing yet. The partner may still be driving in, and the
// rail must not be pulled from this end while it does.
static void holdAtTouch(int a, HomeState touch) {
  stepper[a]->forceStopAndNewPosition(stepper[a]->getCurrentPosition());
  st[a].home = touch;
}

// Leaving a touch is done for the whole pair at once, by whichever end
// notices the pair is complete. Advancing one end at a time would not
// work: the loop services axis 0, moves it on, then tests axis 1 against a
// partner that has already left the touch state.
static void startBackoff(int a) {
  st[a].home = H_BACKOFF_1;
  applyAxisSpeed(a, cfg[a].runSpeed);
  stepper[a]->move(-cfg[a].homeDir * (int32_t)cfg[a].backoffSteps);
  st[a].moveMs = millis();
}

static void startWorkOffset(int a) {
  st[a].home = H_MOVE_TO_ZERO;
  applyAxisSpeed(a, cfg[a].runSpeed);
  stepper[a]->moveTo(-cfg[a].homeDir * (int32_t)cfg[a].workOffset);
  st[a].moveMs = millis();
}

static void serviceHoming(int a) {
  if (!stepper[a]) return;
  FastAccelStepper* s = stepper[a];

  switch (st[a].home) {

    case H_APPROACH_FAST:
      if (stallDetected(a)) {
        holdAtTouch(a, H_TOUCH_1);
      } else if (moveFinished(a)) {
        char msg[48];
        snprintf(msg, sizeof(msg), "no stop in %u steps", cfg[a].maxHomeTravel);
        faultAxis(a, msg);
      }
      break;

    // Holding at the stop, waiting for the other end. An end that never
    // arrives runs out its maxHomeTravel and faults, and faultAxis takes
    // this one down with it, so the wait cannot hang.
    case H_TOUCH_1:
      if (pairAt(a, H_TOUCH_1)) {
        startBackoff(a);
        if (isGantry(a)) startBackoff(partnerOf(a));
      }
      break;

    // Both ends were released together and are running the same relative
    // move at the same speed, so they clear the stop together too.
    case H_BACKOFF_1:
      if (moveFinished(a)) {
        st[a].home = H_APPROACH_SLOW;
        armStallWindow(a);
        applyAxisSpeed(a, cfg[a].homeSpeedSlow);
        s->move(cfg[a].homeDir * (int32_t)(cfg[a].backoffSteps * 3));
        st[a].moveMs = millis();
      }
      break;

    case H_APPROACH_SLOW:
      if (stallDetected(a)) {
        holdAtTouch(a, H_TOUCH_2);
        // The slow second touch is the repeatable one, and each end calls
        // its OWN touch zero - that per-end reference is what squares the
        // rail. Only the move off the stop is shared.
        s->setCurrentPosition(0);
        if (hasEncoder(a)) encoder[a].setCount(0);
        setCmdTarget(a, 0);
      } else if (moveFinished(a)) {
        faultAxis(a, "lost contact on slow approach");
      }
      break;

    case H_TOUCH_2:
      if (pairAt(a, H_TOUCH_2)) {
        startWorkOffset(a);
        if (isGantry(a)) startWorkOffset(partnerOf(a));
      }
      break;

    case H_MOVE_TO_ZERO:
      if (moveFinished(a)) {
        s->setCurrentPosition(0);
        if (hasEncoder(a)) encoder[a].setCount(0);
        setCmdTarget(a, 0);
        st[a].isHomed = true;
        st[a].home = H_DONE;
        outf("# axis %d homed", a + 1);
      }
      break;

    case H_DEADRECKON:
      // Open loop: cannot see the stall, so run the bounded move out and
      // treat wherever we end up as zero. Steps will be lost by design.
      if (moveFinished(a)) {
        s->setCurrentPosition(0);
        applyAxisSpeed(a, cfg[a].runSpeed);
        s->moveTo(-cfg[a].homeDir * (int32_t)cfg[a].workOffset);
        setCmdTarget(a, -cfg[a].homeDir * (int32_t)cfg[a].workOffset);
        st[a].moveMs = millis();
        st[a].isHomed = true;
        st[a].home = H_DONE;
        outf("# axis %d blind-homed (approximate)", a + 1);
      }
      break;

    default: break;
  }
}

// With ALM unwired this is the only way to notice a latched drive fault:
// commanded steps keep going out while encoder counts stop following.
//
// A fresh move's acceleration ramp makes the encoder legitimately lag the
// commanded steps, same as STALL_BLANK_STEPS covers during homing. Without a
// blank window here, the first 100ms sample after any jog/M could span that
// ramp and read as a following error, tripping a false "lost sync" fault.
static void serviceSyncWatch(int a) {
  if (!hasEncoder(a) || !stepper[a]) return;
  if (st[a].home != H_IDLE && st[a].home != H_DONE) return;

  static int32_t  lastPos[NUM_AXES]    = {0,0,0,0};
  static int64_t  lastCnt[NUM_AXES]    = {0,0,0,0};
  static uint32_t lastMs[NUM_AXES]     = {0,0,0,0};
  static bool     wasRunning[NUM_AXES] = {false,false,false,false};

  bool running = stepper[a]->isRunning();
  if (!running) { wasRunning[a] = false; return; }

  uint32_t now = millis();
  if (!wasRunning[a]) {
    wasRunning[a] = true;
    lastPos[a] = stepper[a]->getCurrentPosition();
    lastCnt[a] = encCount(a);
    lastMs[a]  = now;
    return;
  }

  if (now - lastMs[a] < 100) return;
  lastMs[a] = now;

  int32_t pos = stepper[a]->getCurrentPosition();
  int64_t cnt = encCount(a);
  int32_t dSteps = abs(pos - lastPos[a]);
  int64_t dCnt   = llabs(cnt - lastCnt[a]);
  lastPos[a] = pos;
  lastCnt[a] = cnt;

  if (dSteps > 100) {
    st[a].lastError = (float)dSteps * countsPerStep[a] - (float)dCnt;
    if (st[a].lastError > (float)cfg[a].stallThreshold * 3.0f) {
      faultAxis(a, "lost sync - check drive LED");
    }
  }
}

// Next axis at or after 'from' that is allowed to home, else NUM_AXES.
// HOME ALL has to skip disabled axes rather than command them: the chain
// advances on seeing H_DONE, and an axis that never started would never
// reach it, hanging the sequence forever.
static int nextHomingAxis(int from) {
  for (int a = from; a < NUM_AXES; a++)
    if (cfg[a].homeEnable) return a;
  return NUM_AXES;
}

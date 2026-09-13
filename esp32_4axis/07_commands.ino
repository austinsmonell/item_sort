// =====================================================================
// Command parser. One line-based protocol shared by USB serial and the
// WebSocket transport - see onWsEvent() in 08_network.ino.
// =====================================================================
static void printHelp() {
  outLine("# commands (axis is 1-4; 1 and 2 are one gantry, always move together):");
  outLine("#   J <axis> <steps>     jog, negative steps reverses");
  outLine("#   M <axis> <pos>       move to absolute position");
  outLine("#   H <axis>             home one axis (1 or 2 homes both ends)");
  outLine("#   HA                   home all, one at a time");
  outLine("#   Z <axis>             set current position as zero");
  outLine("#   S                    stop all motion");
  outLine("#   EN 1 | EN 0          energise / release all drives (EN alone reports)");
  outLine("#   IDLE <sec>           drop the drives after this long unmoving, 0 = never");
  outLine("#   FD 1 | FD 0          fault-drop: release all drives on any fault (default on)");
  outLine("#   E 1 | E 0            engage / release e-stop (E 1 also cuts drives; EN 1 after E 0)");
  outLine("#   C <axis> <key> <val> set config, then saved to flash");
  outLine("#     keys: run acc hs hss ppr cpr thr bo wo mt dir inv he");
  outLine("#           spm lmin lmax eni ihold");
  outLine("#     spm = steps per metre; lmin/lmax = soft travel limits in mm");
  outLine("#     (the GUI takes those as cm). lmax <= lmin turns the limit");
  outLine("#     off. Limits are always enforced; until the axis is homed or");
  outLine("#     zeroed the window is measured from the power-on position.");
  outLine("#     eni = 1 if a conducting ENA input ENABLES that drive;");
  outLine("#     ihold = 1 keeps the axis live through the idle timeout");
  outLine("#     run acc hs hss bo wo mt dir he spm lmin lmax eni ihold mirror");
  outLine("#     across axes 1+2; ppr cpr thr inv stay per motor");
  outLine("#   ?                    print one status line");
  outLine("#   V 1 | V 0            status streaming on / off");
  outLine("#   T <axis>             self test: report state, then step slowly");
  outLine("#   IP                   wifi address, signal and client count");
  outLine("#   HELP                 this list");
}

static int parseAxis(const String& tok) {
  int a = tok.toInt() - 1;                      // user-facing axes are 1-4
  return (a >= 0 && a < NUM_AXES) ? a : -1;
}

static bool applyConfigKey(int a, const String& key, long val) {
  if      (key == "run") cfg[a].runSpeed        = max(1L, val);
  else if (key == "acc") cfg[a].accel           = max(1L, val);
  else if (key == "hs")  cfg[a].homeSpeed       = max(1L, val);
  else if (key == "hss") cfg[a].homeSpeedSlow   = max(1L, val);
  else if (key == "ppr") cfg[a].pulsesPerRev    = max(1L, val);
  else if (key == "cpr") cfg[a].encCountsPerRev = max(0L, val);
  else if (key == "thr") cfg[a].stallThreshold  = max(0L, val);
  else if (key == "bo")  cfg[a].backoffSteps    = max(0L, val);
  else if (key == "wo")  cfg[a].workOffset      = max(0L, val);
  else if (key == "mt")  cfg[a].maxHomeTravel   = max(1L, val);
  else if (key == "dir") cfg[a].homeDir         = (val >= 0) ? 1 : -1;
  else if (key == "inv") cfg[a].invertDir       = (val != 0) ? 1 : 0;
  else if (key == "he")  cfg[a].homeEnable      = (val != 0) ? 1 : 0;
  else if (key == "spm") cfg[a].stepsPerM        = max(1L, val);
  else if (key == "lmin") cfg[a].limitMinMm      = (int32_t)val;   // may be negative
  else if (key == "lmax") cfg[a].limitMaxMm      = (int32_t)val;
  else if (key == "eni") cfg[a].enaInvert         = (val != 0) ? 1 : 0;
  else if (key == "ihold") cfg[a].idleHold        = (val != 0) ? 1 : 0;
  else return false;

  recomputeRatios();
  applyAxisSpeed(a, cfg[a].runSpeed);
  applyDirection(a);
  applyEnable(a);
  saveConfig(a);
  return true;
}

// The two gantry ends must run and accelerate identically or the rail racks
// on every move, so the motion keys mirror across the pair. The rest stay
// per motor: they describe one drive's own hardware.
static bool isSharedKey(const String& key) {
  return key == "run"  || key == "acc"  || key == "hs"   || key == "hss" ||
         key == "bo"   || key == "wo"   || key == "dir"  || key == "mt"   ||
         key == "he"   || key == "spm"  || key == "lmin" || key == "lmax" ||
         key == "eni"  || key == "ihold";
}

static void handleConfigCmd(int a, String key, long val) {
  key.toLowerCase();
  if (!applyConfigKey(a, key, val)) { outf("# unknown key '%s'", key.c_str()); return; }
  outf("# axis %d %s = %ld (saved)", a + 1, key.c_str(), val);

  if (isGantry(a) && isSharedKey(key)) {
    int p = partnerOf(a);
    applyConfigKey(p, key, val);
    outf("# axis %d %s = %ld (saved, gantry pair)", p + 1, key.c_str(), val);
  }
}

static void execLine(String line) {
  line.trim();
  if (!line.length()) return;

  // split into up to 4 whitespace-separated tokens
  String tok[4];
  int n = 0;
  int i = 0;
  while (i < (int)line.length() && n < 4) {
    while (i < (int)line.length() && line[i] == ' ') i++;
    int s = i;
    while (i < (int)line.length() && line[i] != ' ') i++;
    if (i > s) tok[n++] = line.substring(s, i);
  }
  if (!n) return;

  String cmd = tok[0];
  cmd.toUpperCase();

  if (cmd == "HELP" || cmd == "?H") { printHelp(); return; }

  if (cmd == "?") { emitStatus(); return; }

  if (cmd == "IP") {
    if (!wifiUp) { outLine("# wifi down - serial only"); return; }
    outf("# %s  ip %s  rssi %d dBm  ws clients %u",
         OTA_HOST, WiFi.localIP().toString().c_str(), WiFi.RSSI(), wsClients);
    return;
  }

  if (cmd == "V") { streaming = (n > 1 && tok[1].toInt() != 0);
                    outf("# streaming %s", streaming ? "on" : "off"); return; }

  if (cmd == "S") { stopAll(); outLine("# stopped"); return; }

  if (cmd == "E") {
    bool set = (n > 1) ? (tok[1].toInt() != 0) : true;
    estopActive = set;
    if (set) {
      // An e-stop cuts motor power, it does not just refuse new commands.
      // killDrives() halts everything and releases every axis, idleHold
      // included - the same as a hardware e-stop in the motor supply.
      // drivesEnabled is left off: releasing the e-stop does NOT re-energise
      // (EN 1 is blocked until E 0), so the drives come back only when asked.
      killDrives();
      outLine("# e-stop ENGAGED - motion stopped, drives OFF");
    } else {
      outLine("# e-stop released - drives still OFF, EN 1 to re-energise");
    }
    return;
  }

  // EN with no argument reports rather than guessing which way to toggle:
  // a control that can be told "the other one" over a link that drops is a
  // control that eventually energises a machine nobody is watching.
  if (cmd == "EN") {
    if (n < 2) { outf("# drives %s", drivesEnabled ? "ENABLED" : "DISABLED"); return; }
    if (estopActive && tok[1].toInt() != 0) {
      outLine("# blocked: e-stop engaged"); return;
    }
    setDrives(tok[1].toInt() != 0, false);
    return;
  }

  // FD 1 | FD 0 - fault-drop safety switch. On (the default), any axis fault
  // releases every drive, the same as E 1. Off, a fault still stops motion and
  // latches the axis but leaves the drives energised. Persisted.
  if (cmd == "FD") {
    if (n < 2) { outf("# fault-drop %s", faultKillsDrives ? "ON" : "off"); return; }
    faultKillsDrives = (tok[1].toInt() != 0);
    saveMachine();
    outf("# fault-drop %s (saved)", faultKillsDrives ? "ON" : "off");
    return;
  }

  if (cmd == "IDLE") {
    if (n < 2) {
      if (idleTimeoutS) outf("# idle timeout %u s", idleTimeoutS);
      else              outLine("# idle timeout off");
      return;
    }
    long v = tok[1].toInt();
    idleTimeoutS = (v < 0) ? 0 : (uint32_t)v;
    lastMotionMs = millis();
    saveMachine();
    if (idleTimeoutS) outf("# idle timeout %u s (saved)", idleTimeoutS);
    else              outLine("# idle timeout off (saved)");
    return;
  }

  if (cmd == "HA") {
    if (estopActive) { outLine("# blocked: e-stop engaged"); return; }
    if (!drivesEnabled) { outLine("# blocked: drives disabled - EN 1 first"); return; }
    int first = nextHomingAxis(0);
    if (first >= NUM_AXES) { outLine("# no axis has homing enabled"); return; }
    homeChain = first;
    commandHome(first);
    outLine("# homing all enabled axes in sequence");
    return;
  }

  // everything below needs an axis argument
  if (n < 2) { outLine("# missing axis"); return; }
  int a = parseAxis(tok[1]);
  if (a < 0) { outLine("# axis must be 1-4"); return; }

  if (cmd == "H") {
    if (estopActive) { outLine("# blocked: e-stop engaged"); return; }
    if (!drivesEnabled) { outLine("# blocked: drives disabled - EN 1 first"); return; }
    commandHome(a);
    if (isGantry(a)) outf("# homing gantry pair (axes %d+%d)", GANTRY_A + 1, GANTRY_B + 1);
    else             outf("# homing axis %d", a + 1);
    return;
  }

  if (cmd == "Z") {
    commandZero(a);
    return;
  }

  // Answers the only question that matters when an axis is dead: is the
  // board failing to command it, or is it commanding fine and nothing
  // downstream is listening? Steps slowly enough to be unmistakable and
  // reports the position it actually reached.
  if (cmd == "T") {
    outf("# axis %d: pul=%u dir=%u  stepper=%s",
         a + 1, PUL_PIN[a], DIR_PIN[a], stepper[a] ? "attached" : "NULL (attach failed at boot)");
    outf("# axis %d: estop=%d state=%s busy=%d homed=%d",
         a + 1, estopActive ? 1 : 0, HOME_STATE_NAME[st[a].home],
         busyHoming(a) ? 1 : 0, st[a].isHomed ? 1 : 0);
    outf("# axis %d: run=%u acc=%u ppr=%u inv=%u he=%u",
         a + 1, cfg[a].runSpeed, cfg[a].accel, cfg[a].pulsesPerRev,
         cfg[a].invertDir, cfg[a].homeEnable);
    if (!stepper[a]) { outf("# axis %d: cannot test, no stepper", a + 1); return; }
    if (estopActive) { outf("# axis %d: cannot test, e-stop engaged", a + 1); return; }
    if (!drivesEnabled) { outf("# axis %d: cannot test, drives disabled", a + 1); return; }

    // 200 steps at 200 Hz is one full second of stepping - slow enough to
    // watch the shaft and to meter the pulse line by hand. Clamp it to the
    // soft limit like any other move, and bail if there is no room, so the
    // self-test can never be the thing that drives an axis off the end.
    int32_t p0    = stepper[a]->getCurrentPosition();
    long    probe = clampToLimits(a, (long)p0 + 200);
    if (probe == p0) {
      outf("# axis %d: on the soft limit, no room for the self-test step", a + 1);
      return;
    }
    stepper[a]->setSpeedInHz(200);
    stepper[a]->setAcceleration(1000);
    stepper[a]->moveTo(probe);
    uint32_t t0 = millis();
    while (stepper[a]->isRunning() && millis() - t0 < 4000) {
      if (wifiUp) ws.loop();     // keep the heartbeat alive across the wait
      delay(10);
    }
    int32_t p1 = stepper[a]->getCurrentPosition();
    applyAxisSpeed(a, cfg[a].runSpeed);        // put the working speed back
    setCmdTarget(a, p1);                       // shadow follows the probe move

    outf("# axis %d: commanded 200, position moved %ld in %lu ms",
         a + 1, (long)(p1 - p0), (unsigned long)(millis() - t0));
    if (p1 - p0 == 0)
      outf("# axis %d: engine issued nothing - firmware side", a + 1);
    else
      outf("# axis %d: pulses were generated - if the shaft did not turn, "
           "the fault is the driver or the wiring", a + 1);
    return;
  }

  if (cmd == "J" || cmd == "M") {
    if (estopActive) { outLine("# blocked: e-stop engaged"); return; }
    // Deliberately not an auto-enable. A disabled axis may have been pushed,
    // and re-enabling is where that gets checked and reported - silently
    // doing it under a jog would bury the one message worth reading.
    if (!drivesEnabled) { outLine("# blocked: drives disabled - EN 1 first"); return; }
    if (!stepper[a]) return;
    if (busyHoming(a)) { outf("# axis %d busy homing", a + 1); return; }
    if (n < 3) { outLine("# missing distance"); return; }
    commandMove(a, tok[2].toInt(), cmd == "M");
    return;
  }

  if (cmd == "C") {
    if (n < 4) { outLine("# usage: C <axis> <key> <value>"); return; }
    handleConfigCmd(a, tok[2], tok[3].toInt());
    return;
  }

  outf("# unknown command '%s' - try HELP", cmd.c_str());
}

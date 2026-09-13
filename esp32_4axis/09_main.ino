// =====================================================================
// Setup / loop. Kept in the last tab on purpose: every type, global and
// function used here is already declared by everything above it.
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  loadConfig();
  recomputeRatios();

  // Before anything can step: an output left as a floating input is at the
  // mercy of whatever the opto leaks.
  for (int a = 0; a < NUM_AXES; a++) {
    pinMode(ENA_PIN[a], OUTPUT);
    applyEnable(a);
  }
  lastMotionMs = millis();

  engine.init();
  for (int a = 0; a < NUM_AXES; a++) {
    stepper[a] = engine.stepperConnectToPin(PUL_PIN[a]);
    if (!stepper[a]) { outf("# axis %d: step pin %u failed", a + 1, PUL_PIN[a]); continue; }
    applyDirection(a);
    stepper[a]->setSpeedInHz(cfg[a].runSpeed);
    stepper[a]->setAcceleration(cfg[a].accel);
    stepper[a]->setCurrentPosition(0);
  }

  // GPIO 36/39/34/35 are input-only with no internal pull resistors. The
  // encoder drives both states actively, so none are needed.
  ESP32Encoder::useInternalWeakPullResistors = puType::none;   // older lib: = NONE;
  for (int e = 0; e < NUM_ENCODERS; e++) {
    encoder[e].attachFullQuad(ENC_A_PIN[e], ENC_B_PIN[e]);
    encoder[e].setCount(0);
  }

  setupNetwork();

  outLine("# 4-axis stepper controller ready");
  printHelp();
}

void loop() {
  // --- network --------------------------------------------------------
  // Both are cheap when idle and neither blocks, so they run every pass
  // rather than on a timer: a stop command arriving over WiFi should not
  // wait behind anything.
  if (wifiUp) {
    ws.loop();
    ArduinoOTA.handle();
  }

  // --- serial input ---------------------------------------------------
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) { execLine(buf); buf = ""; }
    } else if (buf.length() < 96) {
      buf += c;
    }
  }

  if (!estopActive) {
    for (int a = 0; a < NUM_AXES; a++) {
      serviceHoming(a);
      serviceSyncWatch(a);
    }

    // HOME ALL chains one axis to the next so only one is ever crashing.
    // The gantry counts as one link: both ends go together, and the chain
    // waits for both before stepping past them.
    if (homeChain >= 0) {
      int  a    = homeChain;
      bool pair = isGantry(a);
      bool bad  = st[a].home == H_FAULT || (pair && st[partnerOf(a)].home == H_FAULT);
      bool done = st[a].home == H_DONE  && (!pair || st[partnerOf(a)].home == H_DONE);
      if (bad) {
        outf("# home-all aborted at axis %d", a + 1);
        homeChain = -1;
      } else if (done) {
        int next = nextHomingAxis(pair ? afterGantry() : a + 1);
        if (next < NUM_AXES) { homeChain = next; commandHome(next); }
        else { outLine("# home-all complete"); homeChain = -1; }
      }
    }
  }

  serviceIdle();

  // --- periodic status ------------------------------------------------
  static uint32_t lastStream = 0;
  if (streaming && millis() - lastStream >= streamMs) {
    lastStream = millis();
    emitStatus();
  }
}

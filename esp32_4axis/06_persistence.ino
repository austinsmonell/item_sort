// =====================================================================
// Persistence (NVS via Preferences)
// =====================================================================
static void saveConfig(int a) {
  prefs.begin("axes", false);
  char key[16];
  snprintf(key, sizeof(key), "a%d", a);
  prefs.putBytes(key, &cfg[a], sizeof(AxisConfig));
  prefs.end();
}

static void saveMachine() {
  prefs.begin("axes", false);
  prefs.putUInt("idle", idleTimeoutS);
  prefs.putBool("faultkill", faultKillsDrives);
  prefs.end();
}

static void loadConfig() {
  prefs.begin("axes", true);
  idleTimeoutS = prefs.getUInt("idle", 300);
  faultKillsDrives = prefs.getBool("faultkill", true);   // default on
  for (int a = 0; a < NUM_AXES; a++) {
    char key[16];
    snprintf(key, sizeof(key), "a%d", a);
    // A blob saved by an older build is a different length. Ignore it
    // rather than part-filling the struct and leaving new fields as junk.
    if (prefs.getBytesLength(key) == sizeof(AxisConfig))
      prefs.getBytes(key, &cfg[a], sizeof(AxisConfig));
  }
  prefs.end();
}

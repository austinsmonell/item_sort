// =====================================================================
// Output. Every reply goes to both transports, so a command typed in the
// serial monitor shows up in the browser and vice versa. That matters more
// than it sounds: with two ways in, a machine that only told one of them
// what it was doing would be worse than one with a single control path.
// =====================================================================
static void outLine(const char* s) {
  Serial.println(s);
  if (wsClients) ws.broadcastTXT(s);
}

static void outf(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  outLine(buf);
}

// =====================================================================
// Status line. One JSON object per line, prefixed with '{' so the GUI can
// tell it apart from the human-readable '#' messages.
// =====================================================================
static void emitStatus() {
  String j = "{\"estop\":";
  j += estopActive ? "true" : "false";
  j += ",\"drives\":";  j += drivesEnabled ? "true" : "false";
  j += ",\"idle\":";    j += idleTimeoutS;
  j += ",\"fdrop\":";   j += faultKillsDrives ? "true" : "false";
  j += ",\"axis\":[";
  for (int a = 0; a < NUM_AXES; a++) {
    if (a) j += ',';
    bool moving = stepper[a] && stepper[a]->isRunning();
    const char* sname = (st[a].home == H_DONE) ? "homed" : HOME_STATE_NAME[st[a].home];
    j += "{\"pos\":";     j += stepper[a] ? stepper[a]->getCurrentPosition() : 0;
    j += ",\"enc\":";     j += (long)encCount(a);
    j += ",\"err\":";     j += String(st[a].lastError, 1);
    j += ",\"moving\":";  j += moving ? "true" : "false";
    j += ",\"state\":\""; j += sname; j += "\"";
    j += ",\"c\":{\"run\":"; j += cfg[a].runSpeed;
    j += ",\"acc\":"; j += cfg[a].accel;
    j += ",\"hs\":";  j += cfg[a].homeSpeed;
    j += ",\"hss\":"; j += cfg[a].homeSpeedSlow;
    j += ",\"ppr\":"; j += cfg[a].pulsesPerRev;
    j += ",\"cpr\":"; j += cfg[a].encCountsPerRev;
    j += ",\"thr\":"; j += cfg[a].stallThreshold;
    j += ",\"bo\":";  j += cfg[a].backoffSteps;
    j += ",\"wo\":";  j += cfg[a].workOffset;
    j += ",\"mt\":";  j += cfg[a].maxHomeTravel;
    j += ",\"dir\":"; j += cfg[a].homeDir;
    j += ",\"inv\":"; j += cfg[a].invertDir;
    j += ",\"he\":";  j += cfg[a].homeEnable;
    j += ",\"spm\":";  j += cfg[a].stepsPerM;
    j += ",\"lmin\":"; j += cfg[a].limitMinMm;
    j += ",\"lmax\":"; j += cfg[a].limitMaxMm;
    j += ",\"eni\":";  j += cfg[a].enaInvert;
    j += ",\"ihold\":"; j += cfg[a].idleHold;
    j += "}";
    // Outside "c" because it is not a setting - it is whether a limit is
    // configured for this axis at all. When true it is always enforced;
    // before the axis is homed or zeroed the window is measured from the
    // power-on position, which "state" (idle vs homed) tells apart.
    j += ",\"lim\":"; j += limitsActive(a) ? "true" : "false";
    j += "}";
  }
  j += "]}";
  outLine(j.c_str());
}

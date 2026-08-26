/* =====================================================================
   esp32_4axis_serial.ino
   ---------------------------------------------------------------------
   ESP32-WROOM-32U  ->  3x HB808C closed-loop + 1x open-loop stepper
   USB serial control. No WiFi, no Bluetooth, no radio brought up at all.

   Drive it either way:
     - Arduino Serial Monitor at 115200, line ending "Newline". Type HELP.
     - stepper_gui.html in Chrome/Edge, which talks to this over Web Serial.

   AXIS PAIRING:
     Axes 1 and 2 are two motors on opposite ends of one rail, so nothing
     may command one without the other. Jog, move-to, zero and home all go
     out to both ends; the motion config keys mirror across the pair; and a
     fault on one end stops the other. Homing drives both ends into their
     own hard stop, which is what squares the rail.

     Check that the two ends move the same way before homing. If they
     fight, flip one with "C 2 inv 1" rather than rewiring.

   WIRING ASSUMPTION (rev 4 - no buffer chip):
     - 5 V common anode: all PUL+ / DIR+ tied to a shared 5 V rail
     - GPIO sinks the minus lines directly -> LOGIC IS INVERTED
       GPIO LOW = optocoupler conducting = signal asserted
     - Encoders EA+/EB+ tapped through a TXS0108E (5 V -> 3V3)
     - ALM not wired

   PULSE POLARITY NOTE:
     FastAccelStepper idles its step pin LOW and pulses HIGH. With the
     inverted hardware above the opto conducts at idle and switches off
     during each pulse. The HB808C is edge triggered so it steps fine,
     but the input LED runs warm at idle. To avoid that, flip the pulse
     trigger edge in the vendor PC software.

   LIBRARIES (Library Manager):
     - FastAccelStepper   by gin66
     - ESP32Encoder       by Kevin Harrington

   FLASH WITH THE MOTOR SUPPLY OFF. Every upload resets the board and
   leaves the unbuffered PUL lines floating, which can emit stray steps.
   ===================================================================== */

#include <FastAccelStepper.h>
#include <ESP32Encoder.h>
#include <Preferences.h>

// ---------------------------------------------------------------------
// Pin map - rev 4. Do not connect GPIO12 (sets flash voltage at boot).
// GPIO5 and GPIO14 carry DIR, never PUL: GPIO14 emits JTAG activity at
// boot which on a pulse line would be real steps.
// ---------------------------------------------------------------------
#define NUM_AXES 4

static const uint8_t PUL_PIN[NUM_AXES] = { 25, 27, 18, 17 };
static const uint8_t DIR_PIN[NUM_AXES] = { 26, 14,  5, 16 };

#define NUM_ENCODERS 3
static const uint8_t ENC_A_PIN[NUM_ENCODERS] = { 36, 34, 32 };  // VP, 34, 32
static const uint8_t ENC_B_PIN[NUM_ENCODERS] = { 39, 35, 33 };  // VN, 35, 33

// ---------------------------------------------------------------------
// Per-axis configuration
// ---------------------------------------------------------------------
struct AxisConfig {
  uint32_t runSpeed;        // steps/s for normal jogging
  uint32_t accel;           // steps/s^2
  uint32_t homeSpeed;       // steps/s for the first homing approach
  uint32_t homeSpeedSlow;   // steps/s for the second, precise approach
  int32_t  homeDir;         // +1 or -1: which way the hard stop lies
  uint32_t pulsesPerRev;    // must match the driver DIP switches
  uint32_t encCountsPerRev; // encoder line count x4 (quadrature)
  uint32_t stallThreshold;  // following error in encoder counts
  uint32_t backoffSteps;    // retreat distance between approaches
  uint32_t workOffset;      // final distance from the stop to call zero
  uint32_t maxHomeTravel;   // give up after this many steps
  uint32_t invertDir;       // 1 = flip this motor's sense (mirrored mount)
};

AxisConfig cfg[NUM_AXES] = {
  // run   accel  home  slow  dir  ppr   cpr   thr  back  offs  maxtrav  inv
  {  2000,  8000,  600,  120,  -1,  2000, 4000,  40,  400,  800, 200000,  0 },  // gantry end A
  {  2000,  8000,  600,  120,  -1,  2000, 4000,  40,  400,  800, 200000,  0 },  // gantry end B
  {  2000,  8000,  600,  120,  -1,  2000, 4000,  40,  400,  800, 200000,  0 },
  {  2000,  8000,  600,  120,  -1,  1600,    0,   0,  400,  800, 200000,  0 }   // open loop
};

// ---------------------------------------------------------------------
// Gantry pair. Axes 1 and 2 (indices 0 and 1) drive opposite ends of the
// same rail: commanding one alone racks it, so every motion path below
// goes through the pairing helpers rather than an axis index directly.
// ---------------------------------------------------------------------
#define GANTRY_A 0
#define GANTRY_B 1

static inline bool isGantry(int a)  { return a == GANTRY_A || a == GANTRY_B; }
static inline int  partnerOf(int a) { return (a == GANTRY_A) ? GANTRY_B : GANTRY_A; }

// First axis past the pair, for the HOME ALL chain.
static inline int afterGantry() { return (GANTRY_A > GANTRY_B ? GANTRY_A : GANTRY_B) + 1; }

// ---------------------------------------------------------------------
// Homing state machine
// ---------------------------------------------------------------------
enum HomeState : uint8_t {
  H_IDLE = 0, H_APPROACH_FAST, H_BACKOFF_1, H_APPROACH_SLOW,
  H_MOVE_TO_ZERO, H_DONE, H_FAULT, H_DEADRECKON
};

static const char* HOME_STATE_NAME[] = {
  "idle", "approach", "backoff", "precise", "offset", "homed", "FAULT", "blind"
};

struct AxisState {
  HomeState home            = H_IDLE;
  bool      isHomed         = false;
  int32_t   homeStartSteps  = 0;
  int64_t   homeStartCounts = 0;
  float     lastError       = 0.0f;
  char      fault[48]       = "";
};

AxisState st[NUM_AXES];

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper[NUM_AXES] = { nullptr, nullptr, nullptr, nullptr };
ESP32Encoder encoder[NUM_ENCODERS];
Preferences prefs;

bool     estopActive = false;
int      homeChain   = -1;      // >=0 while HOME ALL walks the axes in turn
bool     streaming   = true;    // periodic status lines for the GUI
uint32_t streamMs    = 250;

float countsPerStep[NUM_AXES] = { 0, 0, 0, 0 };

static inline bool hasEncoder(int a) { return a < NUM_ENCODERS && cfg[a].encCountsPerRev > 0; }

static void recomputeRatios() {
  for (int a = 0; a < NUM_AXES; a++) {
    countsPerStep[a] = (cfg[a].pulsesPerRev > 0)
      ? (float)cfg[a].encCountsPerRev / (float)cfg[a].pulsesPerRev : 0.0f;
  }
}

static int64_t encCount(int a) {
  if (!hasEncoder(a)) return 0;
  return encoder[a].getCount();
}

// false = direction pin INVERTED, matching common-anode wiring where a LOW
// GPIO asserts the signal. invertDir flips it again for a motor mounted
// mirrored to its partner - the usual case at the far end of a gantry.
static void applyDirection(int a) {
  if (!stepper[a]) return;
  stepper[a]->setDirectionPin(DIR_PIN[a], cfg[a].invertDir != 0);
}

static void applyAxisSpeed(int a, uint32_t speed) {
  if (!stepper[a]) return;
  stepper[a]->setSpeedInHz(speed);
  stepper[a]->setAcceleration(cfg[a].accel);
}

static void stopAll() {
  homeChain = -1;
  for (int a = 0; a < NUM_AXES; a++) {
    if (stepper[a]) stepper[a]->forceStopAndNewPosition(stepper[a]->getCurrentPosition());
    if (st[a].home != H_FAULT && st[a].home != H_DONE) st[a].home = H_IDLE;
  }
}

// A gantry end that faults has to take its partner down with it: one end
// still driving while the other has stopped is exactly how a rail racks.
static void faultAxis(int a, const char* msg) {
  if (st[a].home == H_FAULT) return;
  if (stepper[a]) stepper[a]->forceStopAndNewPosition(stepper[a]->getCurrentPosition());
  st[a].home    = H_FAULT;
  st[a].isHomed = false;
  snprintf(st[a].fault, sizeof(st[a].fault), "%s", msg);
  Serial.printf("# axis %d FAULT: %s\n", a + 1, msg);
  if (isGantry(a)) faultAxis(partnerOf(a), "partner faulted");   // recurses once
}

// =====================================================================
// Homing
// =====================================================================
static void beginHome(int a) {
  if (estopActive || !stepper[a]) return;

  st[a].isHomed = false;
  st[a].fault[0] = '\0';
  st[a].homeStartSteps  = stepper[a]->getCurrentPosition();
  st[a].homeStartCounts = encCount(a);
  st[a].lastError = 0.0f;

  if (!hasEncoder(a)) {
    st[a].home = H_DEADRECKON;
    applyAxisSpeed(a, cfg[a].homeSpeedSlow);
    stepper[a]->move(cfg[a].homeDir * (int32_t)cfg[a].maxHomeTravel);
    return;
  }

  st[a].home = H_APPROACH_FAST;
  applyAxisSpeed(a, cfg[a].homeSpeed);
  stepper[a]->move(cfg[a].homeDir * (int32_t)cfg[a].maxHomeTravel);
}

static bool stallDetected(int a) {
  if (!hasEncoder(a)) return false;
  int32_t stepsMoved = abs(stepper[a]->getCurrentPosition() - st[a].homeStartSteps);
  if (stepsMoved < 50) return false;            // ignore acceleration lag
  int64_t encMoved = llabs(encCount(a) - st[a].homeStartCounts);
  float   expected = (float)stepsMoved * countsPerStep[a];
  st[a].lastError  = expected - (float)encMoved;
  return st[a].lastError > (float)cfg[a].stallThreshold;
}

static void serviceHoming(int a) {
  if (!stepper[a]) return;
  FastAccelStepper* s = stepper[a];

  switch (st[a].home) {

    case H_APPROACH_FAST:
      if (stallDetected(a)) {
        s->forceStopAndNewPosition(s->getCurrentPosition());
        st[a].home = H_BACKOFF_1;
        applyAxisSpeed(a, cfg[a].runSpeed);
        s->move(-cfg[a].homeDir * (int32_t)cfg[a].backoffSteps);
      } else if (!s->isRunning()) {
        char msg[48];
        snprintf(msg, sizeof(msg), "no stop in %u steps", cfg[a].maxHomeTravel);
        faultAxis(a, msg);
      }
      break;

    case H_BACKOFF_1:
      if (!s->isRunning()) {
        st[a].home = H_APPROACH_SLOW;
        st[a].homeStartSteps  = s->getCurrentPosition();
        st[a].homeStartCounts = encCount(a);
        applyAxisSpeed(a, cfg[a].homeSpeedSlow);
        s->move(cfg[a].homeDir * (int32_t)(cfg[a].backoffSteps * 3));
      }
      break;

    case H_APPROACH_SLOW:
      if (stallDetected(a)) {
        s->forceStopAndNewPosition(s->getCurrentPosition());
        // The slow second touch is the repeatable one: call it zero.
        s->setCurrentPosition(0);
        if (hasEncoder(a)) encoder[a].setCount(0);
        st[a].home = H_MOVE_TO_ZERO;
        applyAxisSpeed(a, cfg[a].runSpeed);
        s->moveTo(-cfg[a].homeDir * (int32_t)cfg[a].workOffset);
      } else if (!s->isRunning()) {
        faultAxis(a, "lost contact on slow approach");
      }
      break;

    case H_MOVE_TO_ZERO:
      if (!s->isRunning()) {
        s->setCurrentPosition(0);
        if (hasEncoder(a)) encoder[a].setCount(0);
        st[a].isHomed = true;
        st[a].home = H_DONE;
        Serial.printf("# axis %d homed\n", a + 1);
      }
      break;

    case H_DEADRECKON:
      // Open loop: cannot see the stall, so run the bounded move out and
      // treat wherever we end up as zero. Steps will be lost by design.
      if (!s->isRunning()) {
        s->setCurrentPosition(0);
        applyAxisSpeed(a, cfg[a].runSpeed);
        s->moveTo(-cfg[a].homeDir * (int32_t)cfg[a].workOffset);
        st[a].isHomed = true;
        st[a].home = H_DONE;
        Serial.printf("# axis %d blind-homed (approximate)\n", a + 1);
      }
      break;

    default: break;
  }
}

// With ALM unwired this is the only way to notice a latched drive fault:
// commanded steps keep going out while encoder counts stop following.
static void serviceSyncWatch(int a) {
  if (!hasEncoder(a) || !stepper[a]) return;
  if (st[a].home != H_IDLE && st[a].home != H_DONE) return;
  if (!stepper[a]->isRunning()) return;

  static int32_t  lastPos[NUM_AXES] = {0,0,0,0};
  static int64_t  lastCnt[NUM_AXES] = {0,0,0,0};
  static uint32_t lastMs[NUM_AXES]  = {0,0,0,0};

  uint32_t now = millis();
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

// =====================================================================
// Paired commands. These are the entry points for motion: each one fans
// the command out to both ends of the gantry, so no caller - GUI or serial
// monitor - can move one end on its own.
// =====================================================================
static void issueMove(int a, long v, bool absolute) {
  if (!stepper[a]) return;
  applyAxisSpeed(a, cfg[a].runSpeed);
  if (absolute) stepper[a]->moveTo(v); else stepper[a]->move(v);
}

static void commandMove(int a, long v, bool absolute) {
  issueMove(a, v, absolute);
  if (isGantry(a)) issueMove(partnerOf(a), v, absolute);
}

static void zeroOne(int a) {
  if (!stepper[a]) return;
  stepper[a]->setCurrentPosition(0);
  if (hasEncoder(a)) encoder[a].setCount(0);
  st[a].isHomed  = true;
  st[a].home     = H_IDLE;
  st[a].fault[0] = '\0';
  Serial.printf("# axis %d zeroed\n", a + 1);
}

// Zeroing both ends where they stand keeps whatever squareness they have.
static void commandZero(int a) {
  zeroOne(a);
  if (isGantry(a)) zeroOne(partnerOf(a));
}

// Both ends run their own state machine into their own hard stop at the
// same time: that simultaneous touch is what squares the rail.
static void commandHome(int a) {
  beginHome(a);
  if (isGantry(a)) beginHome(partnerOf(a));
}

static bool busyHoming(int a) {
  HomeState h = st[a].home;
  if (h != H_IDLE && h != H_DONE && h != H_FAULT) return true;
  if (isGantry(a)) {
    HomeState p = st[partnerOf(a)].home;
    if (p != H_IDLE && p != H_DONE && p != H_FAULT) return true;
  }
  return false;
}

// =====================================================================
// Persistence
// =====================================================================
static void saveConfig(int a) {
  prefs.begin("axes", false);
  char key[16];
  snprintf(key, sizeof(key), "a%d", a);
  prefs.putBytes(key, &cfg[a], sizeof(AxisConfig));
  prefs.end();
}

static void loadConfig() {
  prefs.begin("axes", true);
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

// =====================================================================
// Status line. One JSON object per line, prefixed with '{' so the GUI can
// tell it apart from the human-readable '#' messages.
// =====================================================================
static void emitStatus() {
  String j = "{\"estop\":";
  j += estopActive ? "true" : "false";
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
    j += ",\"dir\":"; j += cfg[a].homeDir;
    j += ",\"inv\":"; j += cfg[a].invertDir;
    j += "}}";
  }
  j += "]}";
  Serial.println(j);
}

// =====================================================================
// Command parser
// =====================================================================
static void printHelp() {
  Serial.println(F("# commands (axis is 1-4; 1 and 2 are one gantry, always move together):"));
  Serial.println(F("#   J <axis> <steps>     jog, negative steps reverses"));
  Serial.println(F("#   M <axis> <pos>       move to absolute position"));
  Serial.println(F("#   H <axis>             home one axis (1 or 2 homes both ends)"));
  Serial.println(F("#   HA                   home all, one at a time"));
  Serial.println(F("#   Z <axis>             set current position as zero"));
  Serial.println(F("#   S                    stop all motion"));
  Serial.println(F("#   E 1 | E 0            engage / release e-stop"));
  Serial.println(F("#   C <axis> <key> <val> set config, then saved to flash"));
  Serial.println(F("#     keys: run acc hs hss ppr cpr thr bo wo dir inv"));
  Serial.println(F("#     run acc hs hss bo wo dir mirror across axes 1+2;"));
  Serial.println(F("#     ppr cpr thr inv stay per motor"));
  Serial.println(F("#   ?                    print one status line"));
  Serial.println(F("#   V 1 | V 0            status streaming on / off"));
  Serial.println(F("#   HELP                 this list"));
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
  else if (key == "dir") cfg[a].homeDir         = (val >= 0) ? 1 : -1;
  else if (key == "inv") cfg[a].invertDir       = (val != 0) ? 1 : 0;
  else return false;

  recomputeRatios();
  applyAxisSpeed(a, cfg[a].runSpeed);
  applyDirection(a);
  saveConfig(a);
  return true;
}

// The two gantry ends must run and accelerate identically or the rail racks
// on every move, so the motion keys mirror across the pair. The rest stay
// per motor: they describe one drive's own hardware.
static bool isSharedKey(const String& key) {
  return key == "run" || key == "acc" || key == "hs" || key == "hss" ||
         key == "bo"  || key == "wo"  || key == "dir";
}

static void handleConfigCmd(int a, String key, long val) {
  key.toLowerCase();
  if (!applyConfigKey(a, key, val)) { Serial.printf("# unknown key '%s'\n", key.c_str()); return; }
  Serial.printf("# axis %d %s = %ld (saved)\n", a + 1, key.c_str(), val);

  if (isGantry(a) && isSharedKey(key)) {
    int p = partnerOf(a);
    applyConfigKey(p, key, val);
    Serial.printf("# axis %d %s = %ld (saved, gantry pair)\n", p + 1, key.c_str(), val);
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

  if (cmd == "V") { streaming = (n > 1 && tok[1].toInt() != 0);
                    Serial.printf("# streaming %s\n", streaming ? "on" : "off"); return; }

  if (cmd == "S") { stopAll(); Serial.println(F("# stopped")); return; }

  if (cmd == "E") {
    bool set = (n > 1) ? (tok[1].toInt() != 0) : true;
    estopActive = set;
    if (set) stopAll();
    Serial.printf("# e-stop %s\n", set ? "ENGAGED" : "released");
    return;
  }

  if (cmd == "HA") {
    if (estopActive) { Serial.println(F("# blocked: e-stop engaged")); return; }
    homeChain = 0;
    commandHome(0);
    Serial.println(F("# homing all axes in sequence"));
    return;
  }

  // everything below needs an axis argument
  if (n < 2) { Serial.println(F("# missing axis")); return; }
  int a = parseAxis(tok[1]);
  if (a < 0) { Serial.println(F("# axis must be 1-4")); return; }

  if (cmd == "H") {
    if (estopActive) { Serial.println(F("# blocked: e-stop engaged")); return; }
    commandHome(a);
    if (isGantry(a)) Serial.printf("# homing gantry pair (axes %d+%d)\n", GANTRY_A + 1, GANTRY_B + 1);
    else             Serial.printf("# homing axis %d\n", a + 1);
    return;
  }

  if (cmd == "Z") {
    commandZero(a);
    return;
  }

  if (cmd == "J" || cmd == "M") {
    if (estopActive) { Serial.println(F("# blocked: e-stop engaged")); return; }
    if (!stepper[a]) return;
    if (busyHoming(a)) { Serial.printf("# axis %d busy homing\n", a + 1); return; }
    if (n < 3) { Serial.println(F("# missing distance")); return; }
    commandMove(a, tok[2].toInt(), cmd == "M");
    return;
  }

  if (cmd == "C") {
    if (n < 4) { Serial.println(F("# usage: C <axis> <key> <value>")); return; }
    handleConfigCmd(a, tok[2], tok[3].toInt());
    return;
  }

  Serial.printf("# unknown command '%s' - try HELP\n", cmd.c_str());
}

// =====================================================================
// Setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  loadConfig();
  recomputeRatios();

  engine.init();
  for (int a = 0; a < NUM_AXES; a++) {
    stepper[a] = engine.stepperConnectToPin(PUL_PIN[a]);
    if (!stepper[a]) { Serial.printf("# axis %d: step pin %u failed\n", a + 1, PUL_PIN[a]); continue; }
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

  Serial.println(F("# 4-axis stepper controller ready (serial only, no radio)"));
  printHelp();
}

void loop() {
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
        Serial.printf("# home-all aborted at axis %d\n", a + 1);
        homeChain = -1;
      } else if (done) {
        int next = pair ? afterGantry() : a + 1;
        if (next < NUM_AXES) { homeChain = next; commandHome(next); }
        else { Serial.println(F("# home-all complete")); homeChain = -1; }
      }
    }
  }

  // --- periodic status ------------------------------------------------
  static uint32_t lastStream = 0;
  if (streaming && millis() - lastStream >= streamMs) {
    lastStream = millis();
    emitStatus();
  }
}

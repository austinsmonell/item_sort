/* =====================================================================
   Hardware pin map, per-axis configuration, and every global shared
   across the other tabs.

   THIS MUST STAY A HEADER, INCLUDED FROM esp32_4axis.ino BEFORE THE .ino
   TABS. Arduino concatenates every .ino tab in the sketch folder into one
   translation unit and generates prototypes for every function it finds,
   inserting that whole block in ONE place near the very top of the merged
   file - ABOVE even the sketch's own #include lines, not just above the
   first function. So a function whose signature takes a custom type
   (HomeState) or a library type (WStype_t) gets an auto-generated
   prototype that references that type before anything has declared it,
   and the build fails with "'X' has not been declared" - regardless of
   where the type's real definition/include sits in the file.
   Documented escape hatch: the IDE skips auto-generating a prototype for
   a function that already has one written by hand - BUT its scanner only
   reads the raw .ino tabs, never a header's contents, so that hand-written
   prototype has to live in the .ino tab itself, not here. pairAt() /
   holdAtTouch() (04_homing.ino) and onWsEvent() (08_network.ino) each
   carry one right above their definition; keep that in mind for any new
   function whose signature takes HomeState, WStype_t, or similar.
   ===================================================================== */
#pragma once

// ---------------------------------------------------------------------
// Network. Edit these two before the first flash.
// ---------------------------------------------------------------------
#define WIFI_SSID  "YFI"
#define WIFI_PASS  "luludoges"
#define OTA_HOST   "stepper"          // -> stepper.local, and the OTA port name
#define OTA_PASS   "stepper-ota"      // Arduino IDE asks for this on upload
#define WS_PORT    81

// ---------------------------------------------------------------------
// Pin map - rev 4. Do not connect GPIO12 (sets flash voltage at boot).
// GPIO5 and GPIO14 carry DIR, never PUL: GPIO14 emits JTAG activity at
// boot which on a pulse line would be real steps.
// ---------------------------------------------------------------------
#define NUM_AXES 4

static const uint8_t PUL_PIN[NUM_AXES] = { 25, 27, 18, 17 };
static const uint8_t DIR_PIN[NUM_AXES] = { 26, 14,  5, 16 };

// ENA+ per axis. None of 23/22/21/19 is a strapping pin, so all four are
// safe to drive from reset. They idle LOW, which on this wiring leaves the
// opto dark - the same state as the unwired ENA the drivers ran on before,
// so a board that reboots comes back with the drives live, not dead.
static const uint8_t ENA_PIN[NUM_AXES] = { 23, 22, 21, 19 };

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
  uint32_t homeEnable;      // 0 = refuse to home this axis at all
  // Linear calibration and soft travel limits. Steps per metre rather than
  // per mm so an integer keeps the precision: 10913 is 0.003% off the
  // measured 11000 steps / 1008 mm, which is 0.03 mm over the whole axis.
  // The limits are held in millimetres because that keeps them integral;
  // the GUI shows and takes them as centimetres to one decimal.
  uint32_t stepsPerM;
  int32_t  limitMinMm;
  int32_t  limitMaxMm;
  // ENA polarity, and whether the idle timeout is allowed to drop this axis.
  // enaInvert 0 means a conducting ENA opto DISABLES the drive, which is how
  // these inputs are normally read - unwired ENA is the running state. Flip
  // it for a drive that reads the input the other way round.
  // idleHold 1 keeps an axis energised through the idle timeout: a drive
  // that is holding a load against gravity must never be dropped by a clock.
  uint32_t enaInvert;
  uint32_t idleHold;
};

AxisConfig cfg[NUM_AXES] = {
  // Travel calibration is measured, not derived: the step/rev numbers say
  // nothing about pulley diameter or screw lead, so these come from driving
  // a known distance and putting a rule on it.
  //   axes 1+2   11000 steps = 100.8 cm  ->  10913 steps/m
  //   axis 3     16000 steps = 148.5 cm  ->  10774 steps/m
  //   axis 4     47000 steps =  28.0 cm  -> 167857 steps/m
  // The limits are that measured travel, so a commanded move cannot be told
  // to leave the rail.
  //
  // run   accel  home  slow  dir  ppr   cpr   thr  back  offs  maxtrav  inv  home  spm     lmin  lmax  eni  ihold
  {  2000,  8000,  600,  120,  -1,  2000, 4000,  40,  400,  800, 200000,  0,   1,   10913,  0,    1008, 0,   0 },  // gantry end A
  {  2000,  8000,  600,  120,  -1,  2000, 4000,  40,  400,  800, 200000,  0,   1,   10913,  0,    1008, 0,   0 },  // gantry end B
  {  2000,  8000,  600,  120,  -1,  2000, 4000,  40,  400,  800, 200000,  0,   1,   10774,  0,    1485, 0,   0 },
  // Open loop, NEMA 17 on a 300 mm T8x8 screw: 8 mm lead / 1600 ppr =
  // 200 steps/mm. maxHomeTravel has to be the length of the AXIS, not a
  // round number - with no encoder H_DEADRECKON cannot tell that it has
  // reached the stop, so every step past the end is a stalled motor at
  // full current. 64000 = 320 mm, just past full travel.
  //
  // homeEnable is 0: with no encoder the only way this axis can find a
  // stop is to drive into it and keep pushing, which is a stalled motor at
  // full current for however long is left in maxHomeTravel. Nothing here
  // can detect arrival. Set a limit switch, or zero it by hand with Z 4.
  {  2000,  8000,  600,  120,  -1,  1600,    0,   0,  400,  800,  64000,  0,   0,   167857, 0,    280,  0,   0 }
};

// ---------------------------------------------------------------------
// Homing states.
// ---------------------------------------------------------------------
// H_TOUCH_1 and H_TOUCH_2 are the gantry synchronisation points: an end
// that has found its stop parks there, motor holding, until its partner
// has found its own. No end ever leaves a stop on its own.
enum HomeState : uint8_t {
  H_IDLE = 0, H_APPROACH_FAST, H_TOUCH_1, H_BACKOFF_1, H_APPROACH_SLOW,
  H_TOUCH_2, H_MOVE_TO_ZERO, H_DONE, H_FAULT, H_DEADRECKON
};

static const char* HOME_STATE_NAME[] = {
  "idle", "approach", "touch", "backoff", "precise",
  "touch2", "offset", "homed", "FAULT", "blind"
};

// The names are indexed by the enum, so a state added to one and not the
// other reads off the end of the array in every status line.
static_assert(sizeof(HOME_STATE_NAME) / sizeof(HOME_STATE_NAME[0]) == H_DEADRECKON + 1,
              "HOME_STATE_NAME is out of step with HomeState");

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
// Homing state machine. The HomeState enum itself lives further up.
// ---------------------------------------------------------------------
struct AxisState {
  HomeState home        = H_IDLE;
  bool      isHomed     = false;
  // Baseline for the CURRENT stall-detection window, not for the whole
  // approach - see stallDetected() in 04_homing.ino.
  int32_t   winSteps    = 0;
  int64_t   winCounts   = 0;
  bool      winArmed    = false;
  uint32_t  moveMs      = 0;      // when the current homing move was issued
  float     lastError   = 0.0f;
  char      fault[48]   = "";
};

AxisState st[NUM_AXES];

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper[NUM_AXES] = { nullptr, nullptr, nullptr, nullptr };
ESP32Encoder encoder[NUM_ENCODERS];
Preferences prefs;

bool     drivesEnabled = true;   // what was last ASKED for, machine wide
bool     axisLive[NUM_AXES] = { true, true, true, true };  // what each ENA pin is doing
uint32_t idleTimeoutS  = 300;    // drop the drives after this long unmoving; 0 = never
bool     faultKillsDrives = true;   // on a fault, release every drive (like E 1)
uint32_t lastMotionMs  = 0;
int64_t  enaCounts[NUM_AXES] = { 0, 0, 0, 0 };   // encoder reading when last disabled

// Shadow of where the last accepted command aims each axis, and the millis()
// it was set. issueMove() (05_motion.ino) owns these; anything that moves an
// axis WITHOUT going through issueMove() - homing, S, a fault, Z - resets
// them so the next jog measures from the truth and not from a target that no
// longer applies. See the note above syncTargetIfSettled() in 05_motion.ino
// for why a plain getCurrentPosition() re-sync is not enough on its own.
long     cmdTarget[NUM_AXES]   = { 0, 0, 0, 0 };
uint32_t cmdTargetMs[NUM_AXES] = { 0, 0, 0, 0 };

static void setCmdTarget(int a, long v) {
  cmdTarget[a]   = v;
  cmdTargetMs[a] = millis();
}

bool     estopActive = false;
int      homeChain   = -1;      // >=0 while HOME ALL walks the axes in turn
bool     streaming   = true;    // periodic status lines for the GUI
uint32_t streamMs    = 250;

WebSocketsServer ws(WS_PORT);
uint8_t  wsClients  = 0;        // counted by hand: connectedClients() still
                                 // includes the client being torn down when
                                 // the DISCONNECTED event fires
bool     wifiUp     = false;

// ---------------------------------------------------------------------
// Encoder / step ratio helpers, shared by homing and the sync watch.
// ---------------------------------------------------------------------
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

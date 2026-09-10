/* =====================================================================
   esp32_4axis_serial.ino
   ---------------------------------------------------------------------
   ESP32-WROOM-32U  ->  3x HB808C closed-loop + 1x open-loop stepper
   USB serial control plus WiFi, and wireless reflash over OTA.

   Drive it any of three ways - all speak the same line protocol:
     - Arduino Serial Monitor at 115200, line ending "Newline". Type HELP.
     - stepper_gui.html over Web Serial (USB cable).
     - stepper_gui.html over WiFi, to ws://<board-ip>:81.

   Output is broadcast to every connected transport, so the serial monitor
   and the browser always see the same thing.

   WIFI IS NOT AN E-STOP PATH. A dropped link cannot stop a moving machine
   any faster than its timeout, so the board watches for that itself: if the
   last WebSocket client goes away while an axis is moving, motion stops.
   That is a backstop, not a safety device. Keep a hardware e-stop in the
   motor supply.

   THE -U MODULE HAS NO ANTENNA ON IT. WROOM-32U brings the RF out to a
   U.FL connector instead of a PCB trace. Without a pigtail antenna fitted
   it will associate from a few inches away and nowhere else.

   AXIS PAIRING:
     Axes 1 and 2 are two motors on opposite ends of one rail, so nothing
     may command one without the other. Jog, move-to, zero and home all go
     out to both ends; the motion config keys mirror across the pair; and a
     fault on one end stops the other. Homing drives both ends into their
     own hard stop, which is what squares the rail.

     Check that the two ends move the same way before homing. If they
     fight, flip one with "C 2 inv 1" rather than rewiring.

   WIRING ASSUMPTION (rev 5 - common cathode, no buffer chip):
     - all PUL- / DIR- tied to ground, shared with the ESP32
     - GPIO drives the plus lines directly -> LOGIC IS NORMAL
       GPIO HIGH = optocoupler conducting = signal asserted
     - Encoders EA+/EB+ tapped through a TXS0108E (5 V -> 3V3)
     - ALM not wired

   WHY THIS IS THE RIGHT WAY ROUND. The previous rev tied the plus lines
   to 5 V and sank the minus lines. A 3.3 V GPIO high then left 1.7 V
   across the opto - above the LED forward voltage, so a couple of mA kept
   flowing and a de-asserted input never actually cleared. Edge-triggered
   PUL survived that; a level-sensitive DIR did not, which is what pinned
   an axis to one direction. Common cathode gives a true 0 V off state.

   The cost is drive current: 3.3 V into an input resistor sized for 5 V
   is roughly 8 mA where the driver expected 14 mA. That is inside spec
   for the usual opto inputs. If one driver is marginal, lower its series
   resistor - do not go back to a 5 V anode.

   PULSE POLARITY NOTE:
     FastAccelStepper idles its step pin LOW and pulses HIGH, which is now
     exactly what the hardware wants: dark at idle, conducting during the
     pulse. Nothing to flip in the vendor software any more, and the input
     LEDs no longer run warm doing nothing.

   LIBRARIES (Library Manager):
     - FastAccelStepper   by gin66
     - ESP32Encoder       by Kevin Harrington
     - WebSockets         by Markus Sattler   ("arduinoWebSockets")

   WIFI SETUP: fill in WIFI_SSID / WIFI_PASS below. The first flash has to
   go over USB; after that the board appears in the Arduino IDE under
   Tools > Port as a network port and OTA_PASS unlocks it.

   FLASH WITH THE MOTOR SUPPLY OFF. Every upload resets the board and
   leaves the unbuffered PUL lines floating, which can emit stray steps.
   ===================================================================== */

#include <FastAccelStepper.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>

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
//
// THIS HAS TO STAY ABOVE THE FIRST FUNCTION IN THE SKETCH. The .ino
// preprocessor generates prototypes for every function and inserts them
// immediately before the first function definition it finds - isGantry(),
// just below. A type used in any signature must therefore be declared
// above that point, or the generated prototype references a type that does
// not exist yet: "'HomeState' has not been declared". pairAt() and
// holdAtTouch() both take one.
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
// Homing state machine. The HomeState enum itself lives further up, above
// the first function in the file - see the note there.
// ---------------------------------------------------------------------
struct AxisState {
  HomeState home        = H_IDLE;
  bool      isHomed     = false;
  // Baseline for the CURRENT stall-detection window, not for the whole
  // approach - see stallDetected().
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
// it was set. issueMove() owns these; anything that moves an axis WITHOUT
// going through issueMove() - homing, S, a fault, Z - resets them so the next
// jog measures from the truth and not from a target that no longer applies.
// See the note above issueMove() for why a plain getCurrentPosition() re-sync
// is not enough on its own.
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

// =====================================================================
// Homing
// =====================================================================

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
// =====================================================================
// Soft travel limits
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

// cmdTarget[] / cmdTargetMs[] are declared up with the other globals. The
// library cannot stand in for cmdTarget: getPositionAfterCommandsCompleted()
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

// Next axis at or after 'from' that is allowed to home, else NUM_AXES.
// HOME ALL has to skip disabled axes rather than command them: the chain
// advances on seeing H_DONE, and an axis that never started would never
// reach it, hanging the sequence forever.
static int nextHomingAxis(int from) {
  for (int a = from; a < NUM_AXES; a++)
    if (cfg[a].homeEnable) return a;
  return NUM_AXES;
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

static bool anyMoving() {
  for (int a = 0; a < NUM_AXES; a++)
    if (stepper[a] && stepper[a]->isRunning()) return true;
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

// =====================================================================
// Command parser
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

// =====================================================================
// Network
// =====================================================================
// A WebSocket frame is one command line, byte for byte what the serial
// monitor would take. Keeping one parser for both transports is the whole
// reason the protocol is line based.
static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {

    case WStype_CONNECTED: {
      wsClients++;
      IPAddress ip = ws.remoteIP(num);
      outf("# ws client %u connected from %s", num, ip.toString().c_str());
      emitStatus();                         // new client should not wait for the tick
      break;
    }

    case WStype_DISCONNECTED:
      if (wsClients) wsClients--;
      Serial.printf("# ws client %u disconnected\n", num);
      // Last one out while the machine is moving: the operator has lost
      // sight of it and can no longer press stop, so stop for them. The
      // heartbeat below is what makes this fire on a dead link and not
      // just on a clean browser close.
      if (wsClients == 0 && anyMoving()) {
        stopAll();
        outLine("# link lost with motion in progress - stopped");
      }
      break;

    case WStype_TEXT: {
      String line;
      line.reserve(len + 1);
      for (size_t i = 0; i < len; i++) line += (char)payload[i];
      execLine(line);
      break;
    }

    default: break;
  }
}

static void setupNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // modem sleep adds ~100 ms of jitter to jogs
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(OTA_HOST);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("# wifi connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  // Not fatal. USB serial still drives the machine, so come up either way
  // rather than sitting in a retry loop with the motors unattended.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("# wifi FAILED - serial control only");
    return;
  }

  wifiUp = true;
  Serial.printf("# wifi %s  ip %s  rssi %d\n",
                WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.RSSI());

  if (MDNS.begin(OTA_HOST)) {
    MDNS.addService("ws", "tcp", WS_PORT);
    Serial.printf("# mdns %s.local\n", OTA_HOST);
  }

  ws.begin();
  ws.onEvent(onWsEvent);
  // Ping every 2 s, expect a pong inside 1 s, give up after 2 misses. This
  // is what turns a yanked power lead or a dead AP into a DISCONNECTED
  // event in ~5 s instead of hanging a half-open socket indefinitely.
  ws.enableHeartbeat(2000, 1000, 2);
  Serial.printf("# websocket on ws://%s:%d\n", WiFi.localIP().toString().c_str(), WS_PORT);

  ArduinoOTA.setHostname(OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() {
    // The board reboots at the end of this, so stop the motors while the
    // sketch still can: an axis left running would keep its last commanded
    // direction with nothing servicing the step line. E-stop is engaged to
    // block anything arriving during the transfer, not to persist - the
    // reboot clears it, and it clears isHomed with it. Re-home after OTA.
    // Drop the drives too, same as the E command: nothing should be
    // energised through a reflash.
    killDrives();
    estopActive = true;
    Serial.println("# OTA starting - motion stopped, drives OFF, re-home after reboot");
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("# OTA error %u\n", e); });
  ArduinoOTA.begin();
  Serial.printf("# OTA ready as '%s'\n", OTA_HOST);
}

// =====================================================================
// Setup / loop
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

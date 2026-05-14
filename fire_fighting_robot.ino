/**
 * @file    fire_fighting_robot.ino
 * @author  Raphael Ebubechi Efita
 * @date    2024
 * @version 2.0
 *
 * @brief   Autonomous Fire Detection and Suppression System
 *
 * @details This firmware controls a stationary fire-fighting robot that:
 *            1. Continuously sweeps a flame sensor across a configurable arc
 *            2. Locks onto the direction of a detected flame
 *            3. Aims a water-pump nozzle at the flame source
 *            4. Activates a relay-controlled pump to suppress the fire
 *            5. Verifies suppression before resuming patrol
 *
 *          The control flow is implemented as a two-state Finite State Machine (FSM):
 *            - STATE_SCAN      : Servo sweeps, sensor polled at each step
 *            - STATE_EXTINGUISH: Pump aims and fires; re-checks after each cycle
 *
 * @hardware
 *   - Arduino Uno (ATmega328P, 5 V, 16 MHz)
 *   - IR Flame Sensor  → D2  (active-LOW output)
 *   - 5 V Relay Module → D8  (active-HIGH trigger; pump on when HIGH)
 *   - Scan Servo (SG90)→ D9  (PWM, sweeps flame sensor)
 *   - Pump Servo (SG90)→ D10 (PWM, aims water nozzle)
 *
 * @dfm_notes
 *   All tunable parameters are consolidated in the CONFIGURATION section below.
 *   To adapt this firmware to a different sensor polarity, servo range, or pump
 *   timing, only that section needs to be edited — no logic code changes required.
 *   This separation of configuration from logic is a core DFM principle for
 *   embedded systems, enabling fast field adjustments and safe re-testing.
 *
 * @license MIT
 */

// ─────────────────────────────────────────────────────────────────────────────
// DEPENDENCIES
// ─────────────────────────────────────────────────────────────────────────────
#include <Servo.h>  ///< Standard Arduino servo library (built-in, no install needed)


// ─────────────────────────────────────────────────────────────────────────────
// PIN MAP
// Centralised here so hardware re-wiring never requires hunting through logic.
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t PIN_FLAME_SENSOR = 2;   ///< Digital input: LOW = flame detected
static const uint8_t PIN_PUMP_RELAY   = 8;   ///< Digital output: HIGH = pump ON
static const uint8_t PIN_SCAN_SERVO   = 9;   ///< PWM output: servo carrying flame sensor
static const uint8_t PIN_PUMP_SERVO   = 10;  ///< PWM output: servo aiming water nozzle


// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION  ← DFM: All tunable constants live here, not inside functions.
//                        Adjust these to suit your physical build without
//                        touching any control logic.
// ─────────────────────────────────────────────────────────────────────────────

/** @defgroup ScanConfig  Scan servo sweep parameters */
///@{
static const int SCAN_ANGLE_MIN  = 10;   ///< Leftmost servo position (degrees). Stay > 0 to avoid end-stop strain.
static const int SCAN_ANGLE_MAX  = 170;  ///< Rightmost servo position (degrees). Stay < 180 to avoid end-stop strain.
static const int SCAN_STEP_DEG   = 5;    ///< Angular increment per scan step (degrees). Smaller = finer resolution, slower sweep.
static const int SCAN_STEP_MS    = 100;  ///< Pause at each step (ms). Must allow sensor output to settle before sampling.
///@}

/** @defgroup PumpConfig  Pump and relay timing parameters */
///@{
static const unsigned long PUMP_ON_DURATION_MS   = 3000; ///< How long the pump runs per suppression cycle (ms).
static const unsigned long SERVO_SETTLE_MS        = 500;  ///< Time to wait after commanding a servo before acting (ms).
///@}

/** @defgroup SensorConfig  Flame sensor signal polarity */
///@{
/**
 * @brief  Logic level that indicates fire is PRESENT.
 *
 * Most IR flame sensor modules output LOW when a flame is detected (active-LOW).
 * If your module is active-HIGH, change this to HIGH — no other code changes needed.
 */
static const int FLAME_ACTIVE_LEVEL = LOW;
///@}

/** @defgroup ServoConfig  Neutral/home positions */
///@{
static const int PUMP_SERVO_HOME_ANGLE = 90;  ///< Pump servo resting position (degrees). Centred by default.
///@}

/** @defgroup SerialConfig  Debug serial */
///@{
static const uint32_t SERIAL_BAUD = 9600;  ///< Serial baud rate. Match this in the IDE Serial Monitor.
///@}


// ─────────────────────────────────────────────────────────────────────────────
// FSM STATE DEFINITIONS
// Using an enum makes state transitions explicit and prevents magic-number bugs.
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Operating states of the fire-fighting FSM.
 */
enum RobotState : uint8_t {
  STATE_SCAN,        ///< No fire detected — sensor servo is sweeping
  STATE_EXTINGUISH   ///< Fire detected — pump is active or re-checking
};


// ─────────────────────────────────────────────────────────────────────────────
// GLOBAL STATE  (minimised: only variables that must persist across loop() calls)
// ─────────────────────────────────────────────────────────────────────────────

Servo scanServo;  ///< Controls the servo that carries the flame sensor
Servo pumpServo;  ///< Controls the servo that aims the water nozzle

RobotState currentState = STATE_SCAN;  ///< FSM current state; initialise to scanning

int  scanAngle     = SCAN_ANGLE_MIN;  ///< Current sweep angle (degrees)
int  scanDirection = 1;               ///< Sweep direction: +1 = clockwise, -1 = counter-clockwise
int  fireAngle     = -1;              ///< Angle at which fire was detected; -1 = no fire logged

unsigned long pumpStartTime = 0;  ///< Timestamp (ms) when the pump was last activated


// ─────────────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────────────
void scanForFire();
void extinguishFire();
void activatePump();
void deactivatePump();
bool isFlameDetected();
void advanceScanAngle();
void resetToScanState();
void logState(const char* msg);


// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  One-time hardware initialisation.
 *
 * - Configures GPIO directions and safe initial output states.
 * - Attaches servo objects to their respective PWM pins.
 * - Moves servos to known home positions before the first loop() call.
 *
 * @note   Pump relay is explicitly set LOW (OFF) before servo motion to ensure
 *         the pump never fires unintentionally during power-on transients.
 */
void setup() {
  Serial.begin(SERIAL_BAUD);

  // --- GPIO initialisation ---
  // Relay is set LOW (pump OFF) first — safety measure before anything else runs.
  pinMode(PIN_PUMP_RELAY, OUTPUT);
  digitalWrite(PIN_PUMP_RELAY, LOW);

  pinMode(PIN_FLAME_SENSOR, INPUT);

  // --- Servo initialisation ---
  scanServo.attach(PIN_SCAN_SERVO);
  pumpServo.attach(PIN_PUMP_SERVO);

  // Move servos to home positions and wait for them to physically arrive.
  scanServo.write(SCAN_ANGLE_MIN);
  pumpServo.write(PUMP_SERVO_HOME_ANGLE);
  delay(SERVO_SETTLE_MS * 2);  // Double settle time on startup for reliable homing

  logState("System initialised. Beginning scan.");
}


// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP  — FSM dispatcher
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Main FSM dispatcher.
 *
 * Delegates work to the appropriate handler based on the current FSM state.
 * Adding a new state only requires: (1) a new enum value, (2) a new case here,
 * and (3) a new handler function — no changes to existing logic.
 */
void loop() {
  switch (currentState) {
    case STATE_SCAN:
      scanForFire();
      break;

    case STATE_EXTINGUISH:
      extinguishFire();
      break;

    default:
      // Should never reach here; reset to safe scanning state.
      logState("[WARN] Unknown FSM state. Resetting to SCAN.");
      resetToScanState();
      break;
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// STATE HANDLERS
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  STATE_SCAN handler — sweeps the sensor and transitions on detection.
 *
 * Moves the scan servo one step, waits for the sensor to settle, then reads it.
 * If a flame is detected, the angle is recorded, the pump servo is aimed, the
 * pump is activated, and the FSM transitions to STATE_EXTINGUISH.
 */
void scanForFire() {
  // Command servo to current scan position and wait for mechanical settle.
  scanServo.write(scanAngle);
  delay(SCAN_STEP_MS);

  if (isFlameDetected()) {
    fireAngle = scanAngle;

    Serial.print("[FIRE] Detected at ");
    Serial.print(fireAngle);
    Serial.println(" degrees.");

    // Aim pump nozzle at the fire before activating the pump.
    pumpServo.write(fireAngle);
    delay(SERVO_SETTLE_MS);

    activatePump();
    currentState = STATE_EXTINGUISH;  // FSM transition
    return;  // Exit immediately; do not advance scan angle this cycle.
  }

  advanceScanAngle();
}

/**
 * @brief  STATE_EXTINGUISH handler — monitors pump duration and verifies suppression.
 *
 * Non-blocking check: returns immediately if the pump cycle is still in progress.
 * When the cycle completes, the pump is stopped and the sensor is re-read:
 *   - Fire still present → restart pump cycle.
 *   - Fire gone          → return to STATE_SCAN.
 *
 * @note   Using millis()-based timing (non-blocking) here is intentional.
 *         A blocking delay() inside this function would freeze the MCU and
 *         prevent any future real-time extension (e.g., adding a kill-switch input).
 */
void extinguishFire() {
  // Return early if the pump cycle has not yet elapsed — non-blocking.
  if (millis() - pumpStartTime < PUMP_ON_DURATION_MS) {
    return;
  }

  // Pump cycle complete — stop and re-assess.
  deactivatePump();
  delay(SERVO_SETTLE_MS);  // Brief pause to let water drain before re-reading sensor.

  if (isFlameDetected()) {
    // Flame persists — restart the suppression cycle.
    logState("[FIRE] Flame persists. Restarting suppression cycle.");
    activatePump();
  } else {
    // Flame extinguished — reset system and resume scanning.
    logState("[OK] Flame extinguished. Resuming scan.");
    resetToScanState();
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// HELPER FUNCTIONS
// Small, single-purpose functions improve readability and testability.
// Each one can be unit-tested or swapped out independently.
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Read the flame sensor and return true if a flame is present.
 *
 * Encapsulates the sensor polarity detail (FLAME_ACTIVE_LEVEL).
 * If the sensor is ever swapped for an active-HIGH variant, only the
 * FLAME_ACTIVE_LEVEL constant needs to change — not every call site.
 *
 * @return true   Flame detected at the current servo angle.
 * @return false  No flame detected.
 */
bool isFlameDetected() {
  return digitalRead(PIN_FLAME_SENSOR) == FLAME_ACTIVE_LEVEL;
}

/**
 * @brief  Activate the relay to switch the pump ON and record the start time.
 */
void activatePump() {
  digitalWrite(PIN_PUMP_RELAY, HIGH);
  pumpStartTime = millis();
  logState("[PUMP] Activated.");
}

/**
 * @brief  Deactivate the relay to switch the pump OFF.
 */
void deactivatePump() {
  digitalWrite(PIN_PUMP_RELAY, LOW);
  logState("[PUMP] Deactivated.");
}

/**
 * @brief  Advance the scan servo angle by one step, reversing at arc limits.
 *
 * The servo is clamped to [SCAN_ANGLE_MIN, SCAN_ANGLE_MAX] to prevent
 * mechanical over-rotation and end-stop damage — a DFM-critical guard.
 */
void advanceScanAngle() {
  scanAngle += SCAN_STEP_DEG * scanDirection;

  // Clamp and reverse at arc boundaries.
  if (scanAngle >= SCAN_ANGLE_MAX) {
    scanAngle     = SCAN_ANGLE_MAX;
    scanDirection = -1;  // Reverse to sweep back
  } else if (scanAngle <= SCAN_ANGLE_MIN) {
    scanAngle     = SCAN_ANGLE_MIN;
    scanDirection = 1;   // Reverse to sweep forward
  }
}

/**
 * @brief  Return the system to a clean, safe scanning state.
 *
 * Clears fire data, homes the pump servo, and sets FSM to STATE_SCAN.
 * Calling this function is the only way to leave STATE_EXTINGUISH — all
 * exit paths funnel through here for consistent cleanup.
 */
void resetToScanState() {
  fireAngle     = -1;
  currentState  = STATE_SCAN;

  // Home the pump nozzle before resuming scan to avoid mechanical interference.
  pumpServo.write(PUMP_SERVO_HOME_ANGLE);
  delay(SERVO_SETTLE_MS);
}

/**
 * @brief  Print a status message to the Serial Monitor with a timestamp.
 *
 * Centralising all Serial output through one function means debug logging
 * can be enabled/disabled in one place (e.g., add a DEBUG_MODE flag) without
 * editing every Serial.print() call across the codebase.
 *
 * @param msg  Null-terminated message string to print.
 */
void logState(const char* msg) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.println(msg);
}

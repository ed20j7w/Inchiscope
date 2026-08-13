#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include "DFRobot_GP8403.h"

// Inchiscope Mega firmware -- servo loop only.
//
// All kinematics (constant-curvature model, cubic piston<->bellows inversion),
// all AB diameter<->pressure calibration, and all teleop/inching logic live
// on the PC side (inchiscope_pba_control / inchiscope_ab_control /
// inchiscope_control). This firmware's only job is:
//   - parse ASCII commands from the serial link (PISTON / HOME / VALVE /
//     AB_PID / REG / PING)
//   - home each piston (blind full-retract) before accepting absolute targets
//   - step the 3 piston actuators toward their commanded length
//   - drive each AB's 3-way valve, either from an open-loop duty command or
//     from a firmware-side PID holding a target pressure
//   - drive the 2 shared analog pressure regulators (open-loop -- the
//     regulator hardware itself closes the loop, no sensor on these lines)
//   - read the 3 AB pressure sensors through the I2C mux
//   - push a TEL line at a fixed 20 Hz rate
//
// Serial protocol (see inchiscope_ros2_architecture.md section 3):
//   PC  -> Mega : "PISTON <id> <target_mm>"    id in {p1,p2,p3,d1,d2,d3};
//                   rejected with ERR until that piston has been homed
//                 "HOME <id|ALL>"              blind full-retract homing
//                 "PISTON_RANGE <id|ALL> <min_mm> <max_mm>"  sets this
//                   piston's usable travel; affects clamping AND how long
//                   the next HOME blind-retract runs for
//                 "VALVE <ab_id> <duty_-100_100>" ab_id in {proximal,central,distal};
//                   open-loop 3-way valve position, always switches this AB
//                   to open-loop mode
//                 "AB_PID <ab_id> <target_kpa>" switches this AB to firmware
//                   PID pressure hold; rejected with ERR if that AB's
//                   pressure sensor wasn't detected at boot
//                 "REG POS <kpa>" / "REG NEG <kpa>"  shared analog regulator
//                   setpoints -- open-loop, sent to the I2C DAC that drives
//                   the regulator hardware directly, which self-regulates
//                 "REG_RANGE POS <min_kpa> <max_kpa>" / "REG_RANGE NEG <min_kpa> <max_kpa>"
//                   sets the kPa<->DAC-voltage mapping range for that regulator
//                 "PING"
//   Mega -> PC  : "TEL <t_ms> <p_prox_kpa> <p_cent_kpa> <p_dist_kpa>
//                       <p1_mm> <p2_mm> <p3_mm> <d1_mm> <d2_mm> <d3_mm>
//                       <p1_homed> <p2_homed> <p3_homed> <d1_homed> <d2_homed> <d3_homed>
//                       <prox_mode> <cent_mode> <dist_mode>
//                       <prox_sensor_ok> <cent_sensor_ok> <dist_sensor_ok>"
//                   homed flags are 0/1, mode is "O" (open_loop) or "P" (pid),
//                   sensor_ok flags are 0/1 (detected at boot)
//                 "ACK <command_echo>"
//                 "ERR <message>"

// ---------------------------------------------------------------------------
// Hardware mapping
// ---------------------------------------------------------------------------
//
// NOTE: the current bench hardware wires only ONE PBA segment's worth of
// linear actuators (3 stepper-driven pistons, reusing the pin sets from the
// original main.cpp). The protocol reserves piston ids for BOTH segments
// (p1..p3 = proximal PBA, d1..d3 = distal PBA) so the PC-side code never
// has to change when the second segment is wired in. Until then, p1/p2/p3
// are accepted by the parser but reported as not-connected. If your bench
// setup actually has the proximal segment wired, flip PISTON_CONNECTED /
// the pin tables below.
//
// Each AB has a single 3-way solenoid valve that switches between the shared
// positive (inflate) and negative/vacuum (deflate) regulator lines -- there
// is no separate inflate/deflate valve pair. PWM-ing that one valve pin
// trades off time-connected-to-positive vs time-connected-to-negative, which
// is exactly what the signed open-loop duty (-100..100) and the PID output
// both drive under the hood (see ab_duty_pct below).
//
// The two shared analog regulators are driven over I2C, not PWM: a DFRobot
// GP8403 2-channel 0-10V DAC (SKU DFR0971) at address 0x5F, channel 0 =
// positive/inflate, channel 1 = negative/vacuum -- confirmed from the
// original pre-ROS2 firmware. The regulators themselves are open-loop from
// this firmware's point of view (the DAC voltage IS the pressure setpoint,
// regulator hardware self-regulates), so there's no sensor feedback on
// these two lines, only on the three AB-side sensors downstream of the
// valves. The kPa<->voltage mapping range for each regulator defaults to
// its physical limits (0..100 kPa positive, -100..0 kPa negative) but is
// runtime-settable via REG_RANGE, same as the piston travel range below.

#define MUX_ADDRESS 0x70  // I2C address of the pressure-sensor mux
#define REGULATOR_DAC_ADDRESS 0x5F  // I2C address of the DFRobot GP8403 DAC

const int PISTON_COUNT = 6;
const char* const PISTON_IDS[PISTON_COUNT] = {"p1", "p2", "p3", "d1", "d2", "d3"};

// Only d1..d3 (indices 3..5) are physically wired on current hardware.
const bool PISTON_CONNECTED[PISTON_COUNT] = {false, false, false, true, true, true};

// 4-wire stepper driver pins per connected piston (unused entries left as -1).
const int PISTON_STEP_PINS[PISTON_COUNT][4] = {
  {-1, -1, -1, -1},
  {-1, -1, -1, -1},
  {-1, -1, -1, -1},
  {2, 3, 4, 5},     // d1
  {6, 7, 8, 9},     // d2
  {10, 11, 12, 13}, // d3
};

const int AB_COUNT = 3;
const char* const AB_IDS[AB_COUNT] = {"proximal", "central", "distal"};
const int AB_VALVE_PIN[AB_COUNT] = {23, 24, 25};       // 3-way solenoid valve PWM pins
const uint8_t AB_MUX_CHANNEL[AB_COUNT] = {0, 1, 2};    // mux channel per pressure sensor

const uint8_t REG_POS_DAC_CHANNEL = 0;
const uint8_t REG_NEG_DAC_CHANNEL = 1;

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

const float PISTON_STEP_SIZE_MM = 0.01f;       // mm advanced per stepper microstep
const unsigned long PISTON_STEP_PERIOD_MS = 2;  // ms between stepper steps

// Default usable piston travel -- the full 0..100mm stroke. Runtime-settable
// per piston via PISTON_RANGE (see piston_min_mm/piston_max_mm below), since
// the real usable range depends on how each actuator is mounted/geared.
const float DEFAULT_PISTON_MIN_MM = 0.0f;
const float DEFAULT_PISTON_MAX_MM = 100.0f;

// Blind full-retract homing: drive a piston toward minimum for long enough
// that even a piston that started at its configured max reaches the
// mechanical bottom, with margin for stall/slip. No limit switches on this
// hardware, so this timed retract is the only way to establish a known
// absolute position. Computed per piston at HOME time (see
// computeHomingDurationMs) since the travel range is runtime-settable.
const float HOMING_MARGIN_FACTOR = 1.5f;  // 50% longer than a full-range retract would take

const unsigned long VALVE_PWM_PERIOD_MS = 100;  // 10 Hz duty-cycle PWM period
const unsigned long TELEMETRY_PERIOD_MS = 50;   // 20 Hz telemetry
// Pressure reads and the AB PID loop share this cadence -- 100 Hz is fast
// enough for a reasonably tight pressure hold without saturating the I2C mux.
const unsigned long AB_PID_PERIOD_MS = 10;

// UNTESTED -- do not rely on AB_PID yet. These are unfit placeholder gains,
// and the sign of PID output -> ab_duty_pct -> actual pressure response has
// never been checked against a real 3-way valve + regulator pair (get that
// wrong and it drives away from target_kpa instead of toward it). Verify
// open-loop VALVE control first, then bench-tune these before trusting
// AB_PID unattended -- same PLACEHOLDER status as the AB pressure ceilings /
// diameter curve on the PC side, but with the added risk of an unverified sign.
const float AB_PID_KP = 2.0f;
const float AB_PID_KI = 0.5f;
const float AB_PID_KD = 0.05f;
const float AB_PID_INTEGRAL_LIMIT = 50.0f;  // anti-windup clamp on the integral term

// Default regulator kPa<->DAC-voltage mapping ranges, matching the physical
// limits of the two regulators. Negative-line setpoints are expressed as
// negative kPa (vacuum, below atmospheric). Runtime-settable via REG_RANGE
// (see reg_pos_min_kpa/etc below) in case the physical regulators are ever
// swapped for a different range.
const float DEFAULT_REG_POS_MIN_KPA = 0.0f;
const float DEFAULT_REG_POS_MAX_KPA = 100.0f;
const float DEFAULT_REG_NEG_MIN_KPA = -100.0f;
const float DEFAULT_REG_NEG_MAX_KPA = 0.0f;

Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);
DFRobot_GP8403 regulator_dac(&Wire, REGULATOR_DAC_ADDRESS);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

float piston_length_mm[PISTON_COUNT];
float piston_target_mm[PISTON_COUNT];
float piston_min_mm[PISTON_COUNT];      // runtime-settable via PISTON_RANGE
float piston_max_mm[PISTON_COUNT];      // runtime-settable via PISTON_RANGE
int piston_step_index[PISTON_COUNT];
unsigned long piston_last_step_time[PISTON_COUNT];
bool piston_homed[PISTON_COUNT];
bool piston_homing[PISTON_COUNT];
unsigned long piston_homing_end_ms[PISTON_COUNT];

enum AbMode { AB_MODE_OPEN_LOOP, AB_MODE_PID };

AbMode ab_mode[AB_COUNT];
int ab_duty_pct[AB_COUNT];              // 0-100, internal representation used by serviceValves()
unsigned long ab_pwm_window_start[AB_COUNT];
float ab_pressure_kpa[AB_COUNT];
bool ab_sensor_connected[AB_COUNT];     // detected at boot via mpr.begin()
float ab_pid_target_kpa[AB_COUNT];

bool regulator_connected = false;       // detected at boot via regulator_dac.begin()
float reg_pos_min_kpa = DEFAULT_REG_POS_MIN_KPA;  // runtime-settable via REG_RANGE
float reg_pos_max_kpa = DEFAULT_REG_POS_MAX_KPA;
float reg_neg_min_kpa = DEFAULT_REG_NEG_MIN_KPA;
float reg_neg_max_kpa = DEFAULT_REG_NEG_MAX_KPA;
float ab_pid_integral[AB_COUNT];
float ab_pid_last_error[AB_COUNT];

unsigned long last_telemetry_time = 0;
unsigned long last_ab_pid_time = 0;
String serial_line_buffer = "";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

int findPistonIndex(const String& id) {
  for (int i = 0; i < PISTON_COUNT; i++) {
    if (id.equals(PISTON_IDS[i])) return i;
  }
  return -1;
}

int findAbIndex(const String& id) {
  for (int i = 0; i < AB_COUNT; i++) {
    if (id.equals(AB_IDS[i])) return i;
  }
  return -1;
}

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Maps a signed open-loop duty (-100..100, +100 = fully positive/inflate,
// -100 = fully negative/deflate) to the internal 0-100 "% time connected to
// the positive line" representation serviceValves() drives the 3-way valve
// with -- the same representation the PID output below is converted to.
int signedDutyToInternalPct(int signed_duty) {
  return (signed_duty + 100) / 2;
}

// 4-step full-step sequence, reused verbatim from the original firmware.
void stepMotorUp(const int pins[4], int step) {
  switch (step) {
    case 3: digitalWrite(pins[0], HIGH); digitalWrite(pins[1], LOW);  digitalWrite(pins[2], HIGH); digitalWrite(pins[3], LOW);  break;
    case 2: digitalWrite(pins[0], LOW);  digitalWrite(pins[1], HIGH); digitalWrite(pins[2], HIGH); digitalWrite(pins[3], LOW);  break;
    case 1: digitalWrite(pins[0], LOW);  digitalWrite(pins[1], HIGH); digitalWrite(pins[2], LOW);  digitalWrite(pins[3], HIGH); break;
    case 0: digitalWrite(pins[0], HIGH); digitalWrite(pins[1], LOW);  digitalWrite(pins[2], LOW);  digitalWrite(pins[3], HIGH); break;
  }
}

void stepMotorDown(const int pins[4], int step) {
  switch (step) {
    case 0: digitalWrite(pins[0], HIGH); digitalWrite(pins[1], LOW);  digitalWrite(pins[2], HIGH); digitalWrite(pins[3], LOW);  break;
    case 1: digitalWrite(pins[0], LOW);  digitalWrite(pins[1], HIGH); digitalWrite(pins[2], HIGH); digitalWrite(pins[3], LOW);  break;
    case 2: digitalWrite(pins[0], LOW);  digitalWrite(pins[1], HIGH); digitalWrite(pins[2], LOW);  digitalWrite(pins[3], HIGH); break;
    case 3: digitalWrite(pins[0], HIGH); digitalWrite(pins[1], LOW);  digitalWrite(pins[2], LOW);  digitalWrite(pins[3], HIGH); break;
  }
}

void holdMotor(const int pins[4]) {
  for (int i = 0; i < 4; i++) digitalWrite(pins[i], LOW);
}

// Recomputed at HOME time, not compile time, since piston_min_mm/max_mm are
// runtime-settable via PISTON_RANGE.
unsigned long computeHomingDurationMs(int idx) {
  float travel_mm = piston_max_mm[idx] - piston_min_mm[idx];
  return (unsigned long)((travel_mm / PISTON_STEP_SIZE_MM) * PISTON_STEP_PERIOD_MS *
                          HOMING_MARGIN_FACTOR);
}

void startHoming(int idx) {
  piston_homing[idx] = true;
  piston_homed[idx] = false;
  piston_homing_end_ms[idx] = millis() + computeHomingDurationMs(idx);
}

void servicePistons(unsigned long now) {
  for (int i = 0; i < PISTON_COUNT; i++) {
    if (!PISTON_CONNECTED[i]) continue;
    if (now - piston_last_step_time[i] < PISTON_STEP_PERIOD_MS) continue;
    piston_last_step_time[i] = now;

    const int* pins = PISTON_STEP_PINS[i];
    int step = piston_step_index[i] % 4;

    if (piston_homing[i]) {
      if ((long)(now - piston_homing_end_ms[i]) >= 0) {
        // Long enough to have reached the mechanical bottom even from
        // piston_max_mm[i] -- declare this the zero reference.
        piston_homing[i] = false;
        piston_homed[i] = true;
        piston_length_mm[i] = piston_min_mm[i];
        piston_target_mm[i] = piston_min_mm[i];
        holdMotor(pins);
      } else {
        // Deliberately ignore piston_min_mm[i] here -- actual position is
        // unknown until homing completes, so keep retracting regardless of
        // the (possibly wrong) tracked length.
        stepMotorDown(pins, step);
      }
      piston_step_index[i] = (piston_step_index[i] + 1) % 4;
      continue;
    }

    if (!piston_homed[i]) {
      holdMotor(pins);
      continue;
    }

    float error = piston_target_mm[i] - piston_length_mm[i];
    if (error > PISTON_STEP_SIZE_MM && piston_length_mm[i] < piston_max_mm[i]) {
      stepMotorUp(pins, step);
      piston_length_mm[i] += PISTON_STEP_SIZE_MM;
    } else if (error < -PISTON_STEP_SIZE_MM && piston_length_mm[i] > piston_min_mm[i]) {
      stepMotorDown(pins, step);
      piston_length_mm[i] -= PISTON_STEP_SIZE_MM;
    } else {
      holdMotor(pins);
    }
    piston_step_index[i] = (piston_step_index[i] + 1) % 4;
  }
}

void serviceValves(unsigned long now) {
  for (int i = 0; i < AB_COUNT; i++) {
    unsigned long elapsed = now - ab_pwm_window_start[i];
    if (elapsed >= VALVE_PWM_PERIOD_MS) {
      ab_pwm_window_start[i] = now;
      elapsed = 0;
    }
    unsigned long on_time = (VALVE_PWM_PERIOD_MS * (unsigned long)ab_duty_pct[i]) / 100;
    digitalWrite(AB_VALVE_PIN[i], elapsed < on_time ? HIGH : LOW);
  }
}

void readAbPressures() {
  for (int i = 0; i < AB_COUNT; i++) {
    if (!ab_sensor_connected[i]) continue;
    selectMuxChannel(AB_MUX_CHANNEL[i]);
    ab_pressure_kpa[i] = mpr.readPressure() / 10.0f;
  }
}

void serviceAbPid(float dt_sec) {
  for (int i = 0; i < AB_COUNT; i++) {
    if (ab_mode[i] != AB_MODE_PID) continue;

    float error = ab_pid_target_kpa[i] - ab_pressure_kpa[i];
    ab_pid_integral[i] = constrain(
        ab_pid_integral[i] + error * dt_sec, -AB_PID_INTEGRAL_LIMIT, AB_PID_INTEGRAL_LIMIT);
    float derivative = (dt_sec > 0.0f) ? (error - ab_pid_last_error[i]) / dt_sec : 0.0f;
    ab_pid_last_error[i] = error;

    float output = AB_PID_KP * error + AB_PID_KI * ab_pid_integral[i] + AB_PID_KD * derivative;
    int signed_duty = (int)constrain(output, -100.0f, 100.0f);
    ab_duty_pct[i] = signedDutyToInternalPct(signed_duty);
  }
}

// Maps a kPa setpoint to the GP8403's 0-10V output range, expressed as the
// millivolt argument setDACOutVoltage() expects.
uint16_t mapKpaToDacMillivolts(float kpa, float min_kpa, float max_kpa) {
  float clamped = constrain(kpa, min_kpa, max_kpa);
  float fraction = (clamped - min_kpa) / (max_kpa - min_kpa);
  float voltage = constrain(fraction, 0.0f, 1.0f) * 10.0f;
  return (uint16_t)(voltage * 1000.0f);
}

void publishTelemetry(unsigned long now) {
  Serial.print("TEL ");
  Serial.print(now);
  for (int i = 0; i < AB_COUNT; i++) {
    Serial.print(' ');
    Serial.print(ab_pressure_kpa[i], 2);
  }
  for (int i = 0; i < PISTON_COUNT; i++) {
    Serial.print(' ');
    Serial.print(piston_length_mm[i], 3);
  }
  for (int i = 0; i < PISTON_COUNT; i++) {
    Serial.print(' ');
    Serial.print(piston_homed[i] ? '1' : '0');
  }
  for (int i = 0; i < AB_COUNT; i++) {
    Serial.print(' ');
    Serial.print(ab_mode[i] == AB_MODE_PID ? 'P' : 'O');
  }
  for (int i = 0; i < AB_COUNT; i++) {
    Serial.print(' ');
    Serial.print(ab_sensor_connected[i] ? '1' : '0');
  }
  Serial.println();
}

void handlePistonCommand(const String& args) {
  int sep = args.indexOf(' ');
  if (sep < 0) {
    Serial.print("ERR malformed PISTON: ");
    Serial.println(args);
    return;
  }
  String id = args.substring(0, sep);
  float target = args.substring(sep + 1).toFloat();

  int idx = findPistonIndex(id);
  if (idx < 0) {
    Serial.print("ERR unknown piston id: ");
    Serial.println(id);
    return;
  }
  if (!PISTON_CONNECTED[idx]) {
    Serial.print("ERR piston not connected: ");
    Serial.println(id);
    return;
  }
  if (!piston_homed[idx]) {
    Serial.print("ERR piston not homed: ");
    Serial.println(id);
    return;
  }

  target = constrain(target, piston_min_mm[idx], piston_max_mm[idx]);
  piston_target_mm[idx] = target;

  Serial.print("ACK PISTON ");
  Serial.print(id);
  Serial.print(' ');
  Serial.println(target, 3);
}

void handlePistonRangeCommand(const String& args) {
  int sep = args.indexOf(' ');
  if (sep < 0) {
    Serial.print("ERR malformed PISTON_RANGE: ");
    Serial.println(args);
    return;
  }
  String id = args.substring(0, sep);
  String rest = args.substring(sep + 1);
  int sep2 = rest.indexOf(' ');
  if (sep2 < 0) {
    Serial.print("ERR malformed PISTON_RANGE: ");
    Serial.println(args);
    return;
  }
  float min_mm = rest.substring(0, sep2).toFloat();
  float max_mm = rest.substring(sep2 + 1).toFloat();
  if (min_mm >= max_mm) {
    Serial.print("ERR malformed PISTON_RANGE (min >= max): ");
    Serial.println(args);
    return;
  }

  if (id == "ALL") {
    for (int i = 0; i < PISTON_COUNT; i++) {
      if (!PISTON_CONNECTED[i]) continue;
      piston_min_mm[i] = min_mm;
      piston_max_mm[i] = max_mm;
    }
    Serial.print("ACK PISTON_RANGE ALL ");
    Serial.print(min_mm, 3);
    Serial.print(' ');
    Serial.println(max_mm, 3);
    return;
  }

  int idx = findPistonIndex(id);
  if (idx < 0) {
    Serial.print("ERR unknown piston id: ");
    Serial.println(id);
    return;
  }
  if (!PISTON_CONNECTED[idx]) {
    Serial.print("ERR piston not connected: ");
    Serial.println(id);
    return;
  }

  piston_min_mm[idx] = min_mm;
  piston_max_mm[idx] = max_mm;

  Serial.print("ACK PISTON_RANGE ");
  Serial.print(id);
  Serial.print(' ');
  Serial.print(min_mm, 3);
  Serial.print(' ');
  Serial.println(max_mm, 3);
}

void handleHomeCommand(const String& args) {
  String id = args;
  id.trim();

  if (id == "ALL") {
    for (int i = 0; i < PISTON_COUNT; i++) {
      if (PISTON_CONNECTED[i]) startHoming(i);
    }
    Serial.println("ACK HOME ALL");
    return;
  }

  int idx = findPistonIndex(id);
  if (idx < 0) {
    Serial.print("ERR unknown home target: ");
    Serial.println(id);
    return;
  }
  if (!PISTON_CONNECTED[idx]) {
    Serial.print("ERR piston not connected: ");
    Serial.println(id);
    return;
  }

  startHoming(idx);
  Serial.print("ACK HOME ");
  Serial.println(id);
}

void handleValveCommand(const String& args) {
  int sep = args.indexOf(' ');
  if (sep < 0) {
    Serial.print("ERR malformed VALVE: ");
    Serial.println(args);
    return;
  }
  String id = args.substring(0, sep);
  int duty = args.substring(sep + 1).toInt();

  int idx = findAbIndex(id);
  if (idx < 0) {
    Serial.print("ERR unknown ab id: ");
    Serial.println(id);
    return;
  }

  duty = constrain(duty, -100, 100);
  ab_mode[idx] = AB_MODE_OPEN_LOOP;
  ab_duty_pct[idx] = signedDutyToInternalPct(duty);

  Serial.print("ACK VALVE ");
  Serial.print(id);
  Serial.print(' ');
  Serial.println(duty);
}

void handleAbPidCommand(const String& args) {
  int sep = args.indexOf(' ');
  if (sep < 0) {
    Serial.print("ERR malformed AB_PID: ");
    Serial.println(args);
    return;
  }
  String id = args.substring(0, sep);
  float target_kpa = args.substring(sep + 1).toFloat();

  int idx = findAbIndex(id);
  if (idx < 0) {
    Serial.print("ERR unknown ab id: ");
    Serial.println(id);
    return;
  }
  if (!ab_sensor_connected[idx]) {
    Serial.print("ERR ab pressure sensor not connected: ");
    Serial.println(id);
    return;
  }

  if (ab_mode[idx] != AB_MODE_PID) {
    // Fresh start into PID mode -- reset the controller state so a stale
    // integral/derivative from a previous hold doesn't cause a kick.
    ab_pid_integral[idx] = 0.0f;
    ab_pid_last_error[idx] = 0.0f;
  }
  ab_mode[idx] = AB_MODE_PID;
  ab_pid_target_kpa[idx] = target_kpa;

  Serial.print("ACK AB_PID ");
  Serial.print(id);
  Serial.print(' ');
  Serial.println(target_kpa, 2);
}

void handleRegulatorCommand(const String& args) {
  int sep = args.indexOf(' ');
  if (sep < 0) {
    Serial.print("ERR malformed REG: ");
    Serial.println(args);
    return;
  }
  String which = args.substring(0, sep);
  float target_kpa = args.substring(sep + 1).toFloat();

  if (which != "POS" && which != "NEG") {
    Serial.print("ERR unknown regulator id: ");
    Serial.println(which);
    return;
  }
  if (!regulator_connected) {
    Serial.println("ERR regulator DAC not connected");
    return;
  }

  if (which == "POS") {
    uint16_t mv = mapKpaToDacMillivolts(target_kpa, reg_pos_min_kpa, reg_pos_max_kpa);
    regulator_dac.setDACOutVoltage(mv, REG_POS_DAC_CHANNEL);
    Serial.print("ACK REG POS ");
    Serial.println(target_kpa, 2);
  } else {
    uint16_t mv = mapKpaToDacMillivolts(target_kpa, reg_neg_min_kpa, reg_neg_max_kpa);
    regulator_dac.setDACOutVoltage(mv, REG_NEG_DAC_CHANNEL);
    Serial.print("ACK REG NEG ");
    Serial.println(target_kpa, 2);
  }
}

void handleRegulatorRangeCommand(const String& args) {
  int sep = args.indexOf(' ');
  if (sep < 0) {
    Serial.print("ERR malformed REG_RANGE: ");
    Serial.println(args);
    return;
  }
  String which = args.substring(0, sep);
  String rest = args.substring(sep + 1);
  int sep2 = rest.indexOf(' ');
  if (sep2 < 0) {
    Serial.print("ERR malformed REG_RANGE: ");
    Serial.println(args);
    return;
  }
  float min_kpa = rest.substring(0, sep2).toFloat();
  float max_kpa = rest.substring(sep2 + 1).toFloat();
  if (min_kpa >= max_kpa) {
    Serial.print("ERR malformed REG_RANGE (min >= max): ");
    Serial.println(args);
    return;
  }

  if (which == "POS") {
    reg_pos_min_kpa = min_kpa;
    reg_pos_max_kpa = max_kpa;
  } else if (which == "NEG") {
    reg_neg_min_kpa = min_kpa;
    reg_neg_max_kpa = max_kpa;
  } else {
    Serial.print("ERR unknown regulator id: ");
    Serial.println(which);
    return;
  }

  Serial.print("ACK REG_RANGE ");
  Serial.print(which);
  Serial.print(' ');
  Serial.print(min_kpa, 2);
  Serial.print(' ');
  Serial.println(max_kpa, 2);
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  int sep = line.indexOf(' ');
  String cmd = (sep < 0) ? line : line.substring(0, sep);
  String args = (sep < 0) ? "" : line.substring(sep + 1);

  if (cmd == "PING") {
    Serial.println("ACK PING");
  } else if (cmd == "PISTON") {
    handlePistonCommand(args);
  } else if (cmd == "HOME") {
    handleHomeCommand(args);
  } else if (cmd == "PISTON_RANGE") {
    handlePistonRangeCommand(args);
  } else if (cmd == "VALVE") {
    handleValveCommand(args);
  } else if (cmd == "AB_PID") {
    handleAbPidCommand(args);
  } else if (cmd == "REG") {
    handleRegulatorCommand(args);
  } else if (cmd == "REG_RANGE") {
    handleRegulatorRangeCommand(args);
  } else {
    Serial.print("ERR unknown command: ");
    Serial.println(cmd);
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleLine(serial_line_buffer);
      serial_line_buffer = "";
    } else if (c != '\r') {
      serial_line_buffer += c;
    }
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  for (int i = 0; i < PISTON_COUNT; i++) {
    piston_min_mm[i] = DEFAULT_PISTON_MIN_MM;
    piston_max_mm[i] = DEFAULT_PISTON_MAX_MM;
    piston_length_mm[i] = piston_min_mm[i];
    piston_target_mm[i] = piston_min_mm[i];
    piston_step_index[i] = 0;
    piston_last_step_time[i] = 0;
    piston_homed[i] = false;
    piston_homing[i] = false;
    piston_homing_end_ms[i] = 0;
    if (PISTON_CONNECTED[i]) {
      for (int p = 0; p < 4; p++) {
        pinMode(PISTON_STEP_PINS[i][p], OUTPUT);
      }
    }
  }

  for (int i = 0; i < AB_COUNT; i++) {
    ab_mode[i] = AB_MODE_OPEN_LOOP;
    ab_duty_pct[i] = 0;
    ab_pwm_window_start[i] = 0;
    ab_pid_target_kpa[i] = 0.0f;
    ab_pid_integral[i] = 0.0f;
    ab_pid_last_error[i] = 0.0f;
    pinMode(AB_VALVE_PIN[i], OUTPUT);
    digitalWrite(AB_VALVE_PIN[i], LOW);
  }

  regulator_connected = (regulator_dac.begin() == 0);
  if (regulator_connected) {
    regulator_dac.setDACOutRange(regulator_dac.eOutputRange10V);
    regulator_dac.setDACOutVoltage(0, REG_POS_DAC_CHANNEL);
    regulator_dac.setDACOutVoltage(0, REG_NEG_DAC_CHANNEL);
  } else {
    Serial.println("ERR regulator DAC not found at 0x5F");
  }

  for (int i = 0; i < AB_COUNT; i++) {
    selectMuxChannel(AB_MUX_CHANNEL[i]);
    ab_sensor_connected[i] = mpr.begin();
    if (!ab_sensor_connected[i]) {
      Serial.print("ERR MPRLS sensor not found on channel ");
      Serial.println(AB_MUX_CHANNEL[i]);
    }
  }
}

void loop() {
  unsigned long now = millis();

  pollSerial();
  servicePistons(now);

  if (now - last_ab_pid_time >= AB_PID_PERIOD_MS) {
    float dt_sec = (now - last_ab_pid_time) / 1000.0f;
    last_ab_pid_time = now;
    readAbPressures();
    serviceAbPid(dt_sec);
  }

  serviceValves(now);

  if (now - last_telemetry_time >= TELEMETRY_PERIOD_MS) {
    last_telemetry_time = now;
    publishTelemetry(now);
  }
}

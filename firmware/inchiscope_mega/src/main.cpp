#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"

// Inchiscope Mega firmware — servo loop only.
//
// All kinematics (constant-curvature model, cubic piston<->bellows inversion),
// all AB pressure<->diameter calibration, and all teleop/inching logic have
// moved to the PC side (inchiscope_pba_control / inchiscope_ab_control /
// inchiscope_control). This firmware's only job is:
//   - parse ASCII commands from the serial link (PISTON / VALVE / PING)
//   - step the 3 piston actuators toward their commanded length
//   - PWM the 3 AB solenoid valves toward their commanded duty cycle
//   - read the 3 AB pressure sensors through the I2C mux
//   - push a TEL line at a fixed 20 Hz rate
//
// Serial protocol (see inchiscope_ros2_architecture.md section 3):
//   PC  -> Mega : "PISTON <id> <target_mm>"   id in {p1,p2,p3,d1,d2,d3}
//                 "VALVE <ab_id> <duty_0_100>" ab_id in {proximal,central,distal}
//                 "PING"
//   Mega -> PC  : "TEL <t_ms> <p_prox_kpa> <p_cent_kpa> <p_dist_kpa> <p1_mm> <p2_mm> <p3_mm> <d1_mm> <d2_mm> <d3_mm>"
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

#define MUX_ADDRESS 0x70  // I2C address of the pressure-sensor mux

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
const int AB_VALVE_PIN[AB_COUNT] = {23, 24, 25};       // solenoid valve PWM pins
const uint8_t AB_MUX_CHANNEL[AB_COUNT] = {0, 1, 2};    // mux channel per pressure sensor

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

const float PISTON_STEP_SIZE_MM = 0.01f;       // mm advanced per stepper microstep
const unsigned long PISTON_STEP_PERIOD_MS = 2;  // ms between stepper steps
const float MIN_PISTON_LENGTH_MM = 10.0f;
const float MAX_PISTON_LENGTH_MM = 60.0f;

const unsigned long VALVE_PWM_PERIOD_MS = 100;  // 10 Hz duty-cycle PWM period
const unsigned long TELEMETRY_PERIOD_MS = 50;   // 20 Hz telemetry

Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

float piston_length_mm[PISTON_COUNT];
float piston_target_mm[PISTON_COUNT];
int piston_step_index[PISTON_COUNT];
unsigned long piston_last_step_time[PISTON_COUNT];

int ab_duty_pct[AB_COUNT];
unsigned long ab_pwm_window_start[AB_COUNT];
float ab_pressure_kpa[AB_COUNT];

unsigned long last_telemetry_time = 0;
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

void servicePistons(unsigned long now) {
  for (int i = 0; i < PISTON_COUNT; i++) {
    if (!PISTON_CONNECTED[i]) continue;
    if (now - piston_last_step_time[i] < PISTON_STEP_PERIOD_MS) continue;
    piston_last_step_time[i] = now;

    const int* pins = PISTON_STEP_PINS[i];
    int step = piston_step_index[i] % 4;
    float error = piston_target_mm[i] - piston_length_mm[i];

    if (error > PISTON_STEP_SIZE_MM && piston_length_mm[i] < MAX_PISTON_LENGTH_MM) {
      stepMotorUp(pins, step);
      piston_length_mm[i] += PISTON_STEP_SIZE_MM;
    } else if (error < -PISTON_STEP_SIZE_MM && piston_length_mm[i] > MIN_PISTON_LENGTH_MM) {
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
    selectMuxChannel(AB_MUX_CHANNEL[i]);
    ab_pressure_kpa[i] = mpr.readPressure() / 10.0f;
  }
}

void publishTelemetry(unsigned long now) {
  readAbPressures();
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

  target = constrain(target, MIN_PISTON_LENGTH_MM, MAX_PISTON_LENGTH_MM);
  piston_target_mm[idx] = target;

  Serial.print("ACK PISTON ");
  Serial.print(id);
  Serial.print(' ');
  Serial.println(target, 3);
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

  duty = constrain(duty, 0, 100);
  ab_duty_pct[idx] = duty;

  Serial.print("ACK VALVE ");
  Serial.print(id);
  Serial.print(' ');
  Serial.println(duty);
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
  } else if (cmd == "VALVE") {
    handleValveCommand(args);
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
    piston_length_mm[i] = MIN_PISTON_LENGTH_MM;
    piston_target_mm[i] = MIN_PISTON_LENGTH_MM;
    piston_step_index[i] = 0;
    piston_last_step_time[i] = 0;
    if (PISTON_CONNECTED[i]) {
      for (int p = 0; p < 4; p++) {
        pinMode(PISTON_STEP_PINS[i][p], OUTPUT);
      }
    }
  }

  for (int i = 0; i < AB_COUNT; i++) {
    ab_duty_pct[i] = 0;
    ab_pwm_window_start[i] = 0;
    pinMode(AB_VALVE_PIN[i], OUTPUT);
    digitalWrite(AB_VALVE_PIN[i], LOW);
  }

  for (int i = 0; i < AB_COUNT; i++) {
    selectMuxChannel(AB_MUX_CHANNEL[i]);
    if (!mpr.begin()) {
      Serial.print("ERR MPRLS sensor not found on channel ");
      Serial.println(AB_MUX_CHANNEL[i]);
    }
  }
}

void loop() {
  unsigned long now = millis();

  pollSerial();
  servicePistons(now);
  serviceValves(now);

  if (now - last_telemetry_time >= TELEMETRY_PERIOD_MS) {
    last_telemetry_time = now;
    publishTelemetry(now);
  }
}

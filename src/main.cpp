#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "motorSetup.h"

// Create PWM servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
WebServer server(80);

// =====================================================
// SERVO CONFIGURATION
// =====================================================
#define SERVOMIN     150
#define SERVOMAX     600
#define CENTER       375
#define NUM_SERVOS   8
#define DELAY_TIME   5
#define MAX_STEP     8
#define GAIT_RAMP_TIME_MS 500UL
#define SPOT_HIP_AMPLITUDE 32.0f
#define SPOT_KNEE_LIFT     28.0f

const char WIFI_AP_SSID[] = "RoboDog-ESP32";
const char WIFI_AP_PASSWORD[] = "robodog123";

const int SERVO_TRIM[NUM_SERVOS] = {
  0, 0, 0, 0,
  0, 0, 0, 0
};

const int STAND_OFFSET[NUM_SERVOS] = {
  -50, -50, 0, 50,
  -25, -25, 25, 25
};

const int KNEE_LIFT_SIGN[NUM_SERVOS] = {
  0, 1, 0, -1,
  0, 1, 0, -1
};

const char CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>RoboDog Control</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Arial, Helvetica, sans-serif;
      background: #101418;
      color: #f3f7fa;
    }
    * {
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
    }
    body {
      margin: 0;
      min-height: 100vh;
      background: #101418;
    }
    main {
      width: min(560px, 100%);
      margin: 0 auto;
      padding: 20px 16px 28px;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 18px;
    }
    h1 {
      margin: 0;
      font-size: 28px;
      line-height: 1.05;
      letter-spacing: 0;
    }
    .status {
      min-width: 132px;
      padding: 10px 12px;
      border: 1px solid #2d3742;
      border-radius: 8px;
      background: #161d24;
      text-align: right;
      font-size: 14px;
      color: #b8c5d0;
    }
    .mode {
      display: block;
      margin-top: 3px;
      color: #ffffff;
      font-size: 16px;
      font-weight: 700;
    }
    .pad {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
    }
    button {
      min-height: 72px;
      border: 1px solid #34414d;
      border-radius: 8px;
      background: #202a33;
      color: #ffffff;
      font-size: 18px;
      font-weight: 700;
      touch-action: manipulation;
      cursor: pointer;
    }
    button:active {
      transform: translateY(1px);
      background: #2b3844;
    }
    .primary {
      background: #1f6feb;
      border-color: #3a86ff;
    }
    .stand {
      background: #236b52;
      border-color: #2f8f6f;
    }
    .stop {
      grid-column: 1 / -1;
      min-height: 82px;
      background: #a6252b;
      border-color: #d33b42;
      font-size: 22px;
    }
    .wide {
      grid-column: span 3;
    }
    .spacer {
      visibility: hidden;
    }
    .hint {
      margin: 16px 2px 0;
      color: #9fb0bd;
      font-size: 14px;
      line-height: 1.4;
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>RoboDog</h1>
      <div class="status">Mode<span id="mode" class="mode">Connecting</span></div>
    </header>
    <section class="pad" aria-label="Robot controls">
      <button class="stand wide" onclick="sendCommand('u')">Stand Upright</button>
      <button onclick="sendCommand('w')">Spot Left</button>
      <button class="primary" onclick="sendCommand('f')">Forward</button>
      <button onclick="sendCommand('e')">Spot Right</button>
      <button onclick="sendCommand('l')">Left</button>
      <button class="spacer" tabindex="-1" aria-hidden="true"></button>
      <button onclick="sendCommand('r')">Right</button>
      <button class="stop" onclick="sendCommand('s')">Stop</button>
    </section>
    <p class="hint">Connect to RoboDog-ESP32, then open 192.168.4.1 in your browser.</p>
  </main>
  <script>
    async function sendCommand(command) {
      try {
        const response = await fetch('/cmd?c=' + encodeURIComponent(command), { cache: 'no-store' });
        const data = await response.json();
        document.getElementById('mode').textContent = data.mode;
      } catch (error) {
        document.getElementById('mode').textContent = 'Offline';
      }
    }

    async function updateStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        const data = await response.json();
        document.getElementById('mode').textContent = data.mode;
      } catch (error) {
        document.getElementById('mode').textContent = 'Offline';
      }
    }

    updateStatus();
    setInterval(updateStatus, 1000);
  </script>
</body>
</html>
)rawliteral";

// =====================================================
// GAIT PARAMETERS (Albert-style oscillating gait)
// =====================================================
const float FREQUENCY = 0.006f;
const float AMPLITUDE = 70.0f;

const float phase[NUM_SERVOS] = {
  M_PI + M_PI/8,           M_PI/2 + M_PI + M_PI/8,
  0 + M_PI/8,              M_PI/2 + M_PI/8,
  0 - M_PI/8,              M_PI/2 - M_PI/8,
  M_PI - M_PI/8,           M_PI/2 + M_PI - M_PI/8
};

unsigned long currentMillis = 0;
unsigned long oldMillis = 0;
unsigned long innerTime = 0;
unsigned long modeStartMillis = 0;

// =====================================================
// GLOBAL STATE
// =====================================================
float currentPos[NUM_SERVOS];
float targetPos[NUM_SERVOS];
int lastPulse[NUM_SERVOS];

enum Mode {
  MODE_IDLE,
  MODE_WALK_FORWARD,
  MODE_WALK_LEFT,
  MODE_WALK_RIGHT,
  MODE_SPOT_LEFT,
  MODE_SPOT_RIGHT,
  MODE_UPRIGHT
};
Mode currentMode = MODE_IDLE;

// Function declarations
bool isGaitMode(Mode mode);
void setMode(Mode newMode);
int clampPulse(int pulse);
int calibratedCenter(uint8_t servo);
int standPulse(uint8_t servo);
int stepToward(int current, int target, int maxStep);
float currentGaitRamp();
void writeServoPulse(uint8_t servo, int pulse, bool slewLimit);
void runWalkSequence(const float ampScale[NUM_SERVOS]);
void runSpotTurn(int direction);
void runWalkForward();
void runWalkLeft();
void runWalkRight();
void runWalkSpotLeft();
void runWalkSpotRight();
void runUprightSequence();
void syncCurrentPos();
void moveUntilReachedAll();
const char *modeName(Mode mode);
bool applyCommand(char command);
void processSerialCommand();
void setupWebServer();
void handleWebRoot();
void handleWebCommand();
void handleWebStatus();

void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 1500UL) {
    delay(10);
  }

  Serial.println("RoboDog - Albert Style Gait Demo");
  Serial.println("Commands: 'u' (stand upright), 'f' (forward), 'l' (left), 'r' (right), 'w' (spot-left), 'e' (spot-right), 's' (stop)");

  // Initialize I2C
  Wire.begin();

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  // Initialize all servos to center position
  for (int i = 0; i < NUM_SERVOS; i++) {
    int center = calibratedCenter(i);
    currentPos[i] = center;
    targetPos[i] = center;
    lastPulse[i] = center;
    writeServoPulse(i, center, false);
  }

  currentMillis = millis();
  oldMillis = currentMillis;
  modeStartMillis = currentMillis;

  setupWebServer();

  Serial.println("All servos initialized to center - waiting for command");
  delay(1000);
}

void loop() {
  server.handleClient();
  processSerialCommand();

  switch (currentMode) {
    case MODE_WALK_FORWARD:
      runWalkForward();
      break;
    case MODE_WALK_LEFT:
      runWalkLeft();
      break;
    case MODE_WALK_RIGHT:
      runWalkRight();
      break;
    case MODE_SPOT_LEFT:
      runWalkSpotLeft();
      break;
    case MODE_SPOT_RIGHT:
      runWalkSpotRight();
      break;
    case MODE_UPRIGHT:
      // Hold upright position
      delay(DELAY_TIME);
      break;
    case MODE_IDLE:
    default:
      delay(DELAY_TIME);
      break;
  }

  server.handleClient();
}

void processSerialCommand() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    applyCommand(command);
  }
}

bool applyCommand(char command) {
  command = tolower(command);

  if (command == 'f') {
    setMode(MODE_WALK_FORWARD);
    Serial.println("Walking FORWARD");
  } else if (command == 'l') {
    setMode(MODE_WALK_LEFT);
    Serial.println("Walking LEFT");
  } else if (command == 'r') {
    setMode(MODE_WALK_RIGHT);
    Serial.println("Walking RIGHT");
  } else if (command == 'w') {
    setMode(MODE_SPOT_LEFT);
    Serial.println("Spot turning LEFT");
  } else if (command == 'e') {
    setMode(MODE_SPOT_RIGHT);
    Serial.println("Spot turning RIGHT");
  } else if (command == 'u') {
    syncCurrentPos();
    runUprightSequence();
    setMode(MODE_UPRIGHT);
    Serial.println("Robot standing UPRIGHT");
  } else if (command == 's') {
    setMode(MODE_IDLE);
    syncCurrentPos();
    Serial.println("Stopped - holding current pose");
  } else {
    return false;
  }

  return true;
}

void setupWebServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP started: ");
  Serial.println(WIFI_AP_SSID);
  Serial.print("Password: ");
  Serial.println(WIFI_AP_PASSWORD);
  Serial.print("Open browser: http://");
  Serial.println(ip);

  server.on("/", HTTP_GET, handleWebRoot);
  server.on("/cmd", HTTP_GET, handleWebCommand);
  server.on("/status", HTTP_GET, handleWebStatus);
  server.onNotFound(handleWebRoot);
  server.begin();

  Serial.println("Web control server started");
}

void handleWebRoot() {
  server.send_P(200, "text/html", CONTROL_PAGE);
}

void handleWebCommand() {
  if (!server.hasArg("c") || server.arg("c").length() == 0) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing command\"}");
    return;
  }

  char command = server.arg("c")[0];
  bool ok = applyCommand(command);

  String response = "{\"ok\":";
  response += ok ? "true" : "false";
  response += ",\"mode\":\"";
  response += modeName(currentMode);
  response += "\"}";
  server.send(ok ? 200 : 400, "application/json", response);
}

void handleWebStatus() {
  String response = "{\"mode\":\"";
  response += modeName(currentMode);
  response += "\",\"clients\":";
  response += WiFi.softAPgetStationNum();
  response += "}";
  server.send(200, "application/json", response);
}

const char *modeName(Mode mode) {
  switch (mode) {
    case MODE_WALK_FORWARD:
      return "Forward";
    case MODE_WALK_LEFT:
      return "Left";
    case MODE_WALK_RIGHT:
      return "Right";
    case MODE_SPOT_LEFT:
      return "Spot Left";
    case MODE_SPOT_RIGHT:
      return "Spot Right";
    case MODE_UPRIGHT:
      return "Upright";
    case MODE_IDLE:
    default:
      return "Stopped";
  }
}

bool isGaitMode(Mode mode) {
  return mode == MODE_WALK_FORWARD ||
         mode == MODE_WALK_LEFT ||
         mode == MODE_WALK_RIGHT ||
         mode == MODE_SPOT_LEFT ||
         mode == MODE_SPOT_RIGHT;
}

void setMode(Mode newMode) {
  if (newMode == currentMode) {
    return;
  }

  currentMode = newMode;
  modeStartMillis = millis();

  if (isGaitMode(newMode)) {
    innerTime = 0;
    oldMillis = modeStartMillis;
    currentMillis = modeStartMillis;
  }
}

int clampPulse(int pulse) {
  return constrain(pulse, SERVOMIN, SERVOMAX);
}

int calibratedCenter(uint8_t servo) {
  if (servo >= NUM_SERVOS) {
    return CENTER;
  }

  return clampPulse(CENTER + SERVO_TRIM[servo]);
}

int standPulse(uint8_t servo) {
  if (servo >= NUM_SERVOS) {
    return CENTER;
  }

  return clampPulse(calibratedCenter(servo) + STAND_OFFSET[servo]);
}

int stepToward(int current, int target, int maxStep) {
  int difference = target - current;

  if (abs(difference) <= maxStep) {
    return target;
  }

  return current + (difference > 0 ? maxStep : -maxStep);
}

float currentGaitRamp() {
  unsigned long elapsed = millis() - modeStartMillis;

  if (elapsed >= GAIT_RAMP_TIME_MS) {
    return 1.0f;
  }

  return (float)elapsed / (float)GAIT_RAMP_TIME_MS;
}

void writeServoPulse(uint8_t servo, int pulse, bool slewLimit) {
  pulse = clampPulse(pulse);

  if (slewLimit) {
    pulse = stepToward(lastPulse[servo], pulse, MAX_STEP);
  }

  pwm.setPWM(servo, 0, pulse);
  lastPulse[servo] = pulse;
  currentPos[servo] = (float)pulse;
}

// =====================================================
// GAIT ENGINE — unified walk function with amplitude scaling
// ampScale[]: per-servo amplitude multiplier
//   even indices = hip swing
//   odd  indices = knee lift (rectified, never negative)
// =====================================================
void runWalkSequence(const float ampScale[NUM_SERVOS]) {
  currentMillis = millis();
  innerTime += currentMillis - oldMillis;
  oldMillis = currentMillis;

  float t = FREQUENCY * innerTime;
  float ramp = currentGaitRamp();

  // Hip servos (even indices): full cosine oscillation
  for (int s = 0; s < NUM_SERVOS; s += 2) {
    int pulse = standPulse(s)
              + (int)(ramp * ampScale[s] * AMPLITUDE * cosf(t + phase[s]));
    writeServoPulse(s, pulse, true);
  }

  // Knee servos (odd indices): rectified cosine (never negative)
  for (int s = 1; s < NUM_SERVOS; s += 2) {
    float c = fmaxf(0.0f, cosf(t + phase[s]));
    int pulse = standPulse(s)
              + (int)(ramp * ampScale[s] * AMPLITUDE * c);
    writeServoPulse(s, pulse, true);
  }

  delay(DELAY_TIME);
}

void runSpotTurn(int direction) {
  currentMillis = millis();
  innerTime += currentMillis - oldMillis;
  oldMillis = currentMillis;

  float t = FREQUENCY * innerTime;
  float ramp = currentGaitRamp();

  for (int s = 0; s < NUM_SERVOS; s += 2) {
    int pulse = standPulse(s)
              + (int)(ramp * direction * SPOT_HIP_AMPLITUDE * cosf(t + phase[s]));
    writeServoPulse(s, pulse, true);
  }

  for (int s = 1; s < NUM_SERVOS; s += 2) {
    float c = fmaxf(0.0f, cosf(t + phase[s]));
    int pulse = standPulse(s)
              + (int)(ramp * KNEE_LIFT_SIGN[s] * SPOT_KNEE_LIFT * c);
    writeServoPulse(s, pulse, true);
  }

  delay(DELAY_TIME);
}

// Gait sequence definitions with different amplitude scales
void runWalkForward() {
  static const float a[NUM_SERVOS] = { 1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f };
  runWalkSequence(a);
}

void runWalkLeft() {
  static const float a[NUM_SERVOS] = { 0.2f, 0.2f, -1.5f, -1.5f, 0.2f, 0.2f, -1.5f, -1.5f };
  runWalkSequence(a);
}

void runWalkRight() {
  static const float a[NUM_SERVOS] = { 1.5f, 1.5f, -0.2f, -0.2f, 1.5f, 1.5f, -0.2f, -0.2f };
  runWalkSequence(a);
}

void runWalkSpotLeft() {
  runSpotTurn(1);
}

void runWalkSpotRight() {
  runSpotTurn(-1);
}

// Sync current position from last pulse written by gait engine
void syncCurrentPos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPos[i] = (float)lastPulse[i];
    targetPos[i] = (float)lastPulse[i];
  }
}

// Blocking synchronized movement - all servos interpolate to target simultaneously
void moveUntilReachedAll() {
  const int MOVE_STEPS = 40;
  float startPos[NUM_SERVOS];

  for (int i = 0; i < NUM_SERVOS; i++) {
    startPos[i] = currentPos[i];
  }

  for (int step = 1; step <= MOVE_STEPS; step++) {
    float t = (float)step / (float)MOVE_STEPS;   // 0.0 → 1.0

    for (int i = 0; i < NUM_SERVOS; i++) {
      int pulse = (int)(startPos[i] + t * (targetPos[i] - startPos[i]));
      writeServoPulse(i, pulse, false);
    }
    delay(DELAY_TIME);
  }

  // Snap to exact target
  for (int i = 0; i < NUM_SERVOS; i++) {
    writeServoPulse(i, (int)targetPos[i], false);
  }
}

// Set robot to a stable upright standing position
void runUprightSequence() {
  Serial.println("Moving to UPRIGHT position...");

  for (int i = 0; i < NUM_SERVOS; i++) {
    targetPos[i] = standPulse(i);
  }

  moveUntilReachedAll();
  Serial.println("UPRIGHT position reached!");
}

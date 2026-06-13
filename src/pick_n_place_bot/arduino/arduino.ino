#include <Servo.h>

// ── Servo objects ──────────────────────────────────
Servo base_x;
Servo base_y;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo gripper;

// ── Pin assignments ────────────────────────────────
const int PIN_BASE_X   = 3;
const int PIN_BASE_Y   = 10;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW    = 6;
const int PIN_WRIST    = 7;
const int PIN_GRIPPER  = 8;

// ── Current angle tracking ─────────────────────────
int angleBaseX     = 40;
int angleBaseY     = 90;
int angleShoulder = 160;
int angleElbow    = 50;
int angleWrist    = 0;
int angleGripper  = 180;

// ── Helpers ────────────────────────────────────────
void goHome() {
  base_x.write(angleBaseX);     
  base_y.write(angleBaseY);
  shoulder.write(angleShoulder); 
  elbow.write(angleElbow);    
  wrist.write(angleWrist);    
  gripper.write(angleGripper);
}

void printStatus() {
  Serial.print("JOINTS ");
  Serial.print(angleBaseX);     Serial.print(",");
  Serial.print(angleBaseY);     Serial.print(",");
  Serial.print(angleShoulder);  Serial.print(",");
  Serial.print(angleElbow);     Serial.print(",");
  Serial.print(angleWrist);     Serial.print(",");
  Serial.println(angleGripper);
}

// Clamp angle to safe servo range 0–180
int clamp(int angle) {
  if (angle < 0)   return 0;
  if (angle > 180) return 180;
  return angle;
}

// ── Parse and execute a single command string ─────
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "home") {
    goHome();
    printStatus();
    return;
  }

  if (cmd == "status") {
    printStatus();
    return;
  }

  // Expect format:  <name>-<angle>
  int dashIdx = cmd.indexOf('-');
  if (dashIdx == -1) {
    Serial.println("ERR: unknown command. Use e.g. base-90 or 'home'");
    return;
  }

  String name  = cmd.substring(0, dashIdx);
  int    angle = clamp(cmd.substring(dashIdx + 1).toInt());

  if (name == "base_x") {
    base_x.write(angle);
    angleBaseX = angle;
  } else if (name == "base_y") {
    base_y.write(angle);
    angleBaseY = angle;
  } else if (name == "shoulder") {
    shoulder.write(angle);
    angleShoulder = angle;
  } else if (name == "elbow") {
    elbow.write(angle);
    angleElbow = angle;
  } else if (name == "wrist") {
    wrist.write(angle);
    angleWrist = angle;
  } else if (name == "gripper") {
    // gripper.write(angle);
    gripper.write(angle);
    angleGripper = angle;
  } else {
    Serial.print("ERR: unknown servo '");
    Serial.print(name);
    Serial.println("'. Valid: base_x, base_y, shoulder, elbow, wrist, gripper");
    return;
  }

  printStatus();
}

// ── Split on ';' and dispatch each command in order ──
void handleCommands(String input) {
  input.trim();
  int start = 0;
  while (start < (int)input.length()) {
    int semi  = input.indexOf(';', start);
    String token = (semi == -1)
                     ? input.substring(start)
                     : input.substring(start, semi);
    if (token.length() > 0) handleCommand(token);
    if (semi == -1) break;
    start = semi + 1;
  }
}

// ── Setup ──────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  base_x.attach(PIN_BASE_X);
  base_y.attach(PIN_BASE_Y);
  shoulder.attach(PIN_SHOULDER);
  elbow.attach(PIN_ELBOW);
  wrist.attach(PIN_WRIST);
  gripper.attach(PIN_GRIPPER);

  goHome();
}

// ── Loop ───────────────────────────────────────────
String inputBuffer = "";

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommands(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

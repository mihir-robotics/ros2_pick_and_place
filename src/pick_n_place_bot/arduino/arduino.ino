/*
 * =====================================================
 *  Servo Controller — Arduino NANO
 *  Pins : 3(base) | 5(shoulder) | 7(elbow) | 9(wrist) | 11(gripper)
 *  Home : all servos at 90°
 *
 *  Serial commands (9600 baud, newline-terminated):
 *    base-<angle>      e.g.  base-90
 *    shoulder-<angle>  e.g.  shoulder-45
 *    elbow-<angle>     e.g.  elbow-120
 *    wrist-<angle>     e.g.  wrist-100
 *    gripper-<angle>   e.g.  gripper-0
 *    home              → resets all servos to 90°
 *    status            → prints current angles
 *
 *  Multiple commands can be chained with semicolons:
 *    e.g.  base-90;elbow-45;wrist-120
 * =====================================================
 */

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
const int PIN_BASE_Y   = 11;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW    = 6;
const int PIN_WRIST    = 7;
const int PIN_GRIPPER  = 8;

// ── Current angle tracking ─────────────────────────
int angleBaseX     = 90;
int angleBaseY     = 90;
int angleShoulder = 90;
int angleElbow    = 90;
int angleWrist    = 90;
int angleGripper  = 90;

// ── Helpers ────────────────────────────────────────
void goHome() {
  base_x.write(40);     
  angleBaseX = 40;
  base_y.write(90);     
  angleBaseY = 90;
  shoulder.write(160); 
  angleShoulder = 160;
  elbow.write(50);    
  angleElbow    = 45;
  wrist.write(0);    
  angleWrist    = 0;  
  gripper.write(180);
  angleGripper  = 180;
}

void printStatus() {
  Serial.println("── Current angles ──────────────");
  Serial.print("  base X    : "); Serial.println(angleBaseX);
  Serial.print("  base Y     : "); Serial.println(angleBaseY);
  Serial.print("  shoulder : "); Serial.println(angleShoulder);
  Serial.print("  elbow    : "); Serial.println(angleElbow);
  Serial.print("  wrist    : "); Serial.println(angleWrist);
  Serial.print("  gripper  : "); Serial.println(angleGripper);
  Serial.println("────────────────────────────────");
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
    Serial.println("'. Valid: base, shoulder, elbow, wrist, gripper");
    return;
  }

  Serial.print("OK: ");
  Serial.print(name);
  Serial.print(" → ");
  Serial.print(angle);
  Serial.println("°");
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

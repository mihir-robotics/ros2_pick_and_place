#include <Servo.h>

// ── Servo objects ──────────────────────────────────
Servo base_x;
Servo base_y;
Servo shoulder;
Servo wrist;
Servo gripper;

// ── Pin assignments ────────────────────────────────
const int PIN_BASE_X   = 3;
const int PIN_BASE_Y   = 5;
const int PIN_SHOULDER = 7;
const int PIN_WRIST    = 9;
const int PIN_GRIPPER  = 10;

const int NUM_JOINTS = 5;
const unsigned long MIN_TRAJ_MS = 50;

// ── Current angle tracking ─────────────────────────
int angleBaseX     = 90;
int angleBaseY     = 90;
int angleShoulder  = 90;
int angleWrist     = 90;
int angleGripper   = 180;

Servo *const servos[] = {&base_x, &base_y, &shoulder, &wrist, &gripper};
int *const angles[] = {&angleBaseX, &angleBaseY, &angleShoulder,
                       &angleWrist, &angleGripper};

const int homeAngles[] = {90, 90, 90, 90, 180};

// ── Trajectory state ───────────────────────────────
struct ActiveTraj {
  bool active;
  unsigned long t0;
  unsigned long duration_ms;
  int start[NUM_JOINTS];
  int target[NUM_JOINTS];
  uint8_t mask;
};

ActiveTraj activeTraj = {false, 0, 0, {0}, {0}, 0};

// ── Command queue (one line may contain multiple ';'-separated commands) ──
const int MAX_QUEUE = 8;
String cmdQueue[MAX_QUEUE];
int queueLen = 0;
bool commandBusy = false;

void dispatchCommand(const String &cmdRaw); // ASK: why is this function declared here?

// ── Helpers ────────────────────────────────────────
int clamp(int angle) {
  if (angle < 0)   return 0;
  if (angle > 180) return 180;
  return angle;
}

void writeJoint(int idx, int angle) {
  angle = clamp(angle);
  servos[idx]->write(angle);
  *angles[idx] = angle;
}

void printStatus() {
  Serial.print("JOINTS ");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (i > 0) Serial.print(",");
    Serial.print(*angles[i]);
  }
  Serial.println();
}

void signalDone() {
  Serial.println("DONE");
  commandBusy = false;

  if (queueLen > 0) {
    for (int i = 0; i < queueLen - 1; ++i) {
      cmdQueue[i] = cmdQueue[i + 1];
    }
    --queueLen;
  }

  if (queueLen > 0) {
    dispatchCommand(cmdQueue[0]);
  }
}

void startTrajectory(int targets[], uint8_t mask, unsigned long duration_ms) {
  if (duration_ms < MIN_TRAJ_MS) {
    duration_ms = MIN_TRAJ_MS;
  }

  activeTraj.active = true;
  activeTraj.t0 = millis();
  activeTraj.duration_ms = duration_ms;
  activeTraj.mask = mask;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    activeTraj.start[i] = *angles[i];
    activeTraj.target[i] = targets[i];
  }
}

bool jointNameToIndex(const String &name, int &idx) {
  if (name == "base_x")   { idx = 0; return true; }
  if (name == "base_y")   { idx = 1; return true; }
  if (name == "shoulder") { idx = 2; return true; }
  if (name == "wrist")    { idx = 3; return true; }
  if (name == "gripper")  { idx = 4; return true; }
  return false;
}

void goHomeInstant() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    writeJoint(i, homeAngles[i]);
  }
}

void goHomeTrajectory(unsigned long duration_ms) {
  int targets[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    targets[i] = homeAngles[i];
  }
  startTrajectory(targets, 0x1F, duration_ms);
}

void moveJointInstant(int idx, int angle) {
  writeJoint(idx, angle);
}

void moveJointTrajectory(int idx, int angle, unsigned long duration_ms) {
  int targets[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    targets[i] = *angles[i];
  }
  targets[idx] = clamp(angle);
  startTrajectory(targets, (uint8_t)(1 << idx), duration_ms);
}

bool parseJointCommand(const String &cmd, int &idx, int &angle, unsigned long &duration_ms) {
  int dashIdx = cmd.indexOf('-');
  if (dashIdx == -1) {
    return false;
  }

  String name = cmd.substring(0, dashIdx);
  String rest = cmd.substring(dashIdx + 1);

  int lastDash = rest.lastIndexOf('-');
  if (lastDash != -1) {
    String durationStr = rest.substring(lastDash + 1);
    bool allDigits = durationStr.length() > 0;
    for (unsigned int i = 0; i < durationStr.length() && allDigits; ++i) {
      if (!isDigit(durationStr.charAt(i))) {
        allDigits = false;
      }
    }
    if (allDigits) {
      angle = clamp(rest.substring(0, lastDash).toInt());
      duration_ms = (unsigned long)durationStr.toInt();
      return jointNameToIndex(name, idx);
    }
  }

  angle = clamp(rest.toInt());
  duration_ms = 0;
  return jointNameToIndex(name, idx);
}

void handlePoseCommand(const String &cmd) {
  // Format: pose-<5 comma-separated angles>;<duration_ms>
  int lastSemi = cmd.lastIndexOf(';');
  if (lastSemi == -1) {
    Serial.println("ERR: pose command needs duration, e.g. pose-40,50,0,0,180;2000");
    signalDone();
    return;
  }

  String durationStr = cmd.substring(lastSemi + 1);
  durationStr.trim();
  unsigned long duration_ms = (unsigned long)durationStr.toInt();
  if (duration_ms == 0) {
    Serial.println("ERR: pose duration must be > 0 ms");
    signalDone();
    return;
  }

  String values = cmd.substring(5, lastSemi);  // skip "pose-"
  values.trim();

  int targets[NUM_JOINTS];
  int start = 0;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int comma = values.indexOf(',', start);
    String token = (comma == -1) ? values.substring(start) : values.substring(start, comma);
    token.trim();
    targets[i] = clamp(token.toInt());
    if (comma == -1 && i < NUM_JOINTS - 1) {
      Serial.println("ERR: pose requires 5 comma-separated angles");
      signalDone();
      return;
    }
    if (comma == -1) break;
    start = comma + 1;
  }

  startTrajectory(targets, 0x1F, duration_ms);
}

void dispatchCommand(const String &cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  cmd.toLowerCase();
  commandBusy = true;

  if (cmd == "status") {
    printStatus();
    signalDone();
    return;
  }

  if (cmd == "home") {
    goHomeInstant();
    signalDone();
    return;
  }

  if (cmd.startsWith("home-")) {
    unsigned long duration_ms = (unsigned long)cmd.substring(5).toInt();
    if (duration_ms == 0) {
      Serial.println("ERR: home duration must be > 0 ms");
      signalDone();
      return;
    }
    goHomeTrajectory(duration_ms);
    return;
  }

  if (cmd.startsWith("pose-")) {
    handlePoseCommand(cmd);
    return;
  }

  int idx = 0;
  int angle = 0;
  unsigned long duration_ms = 0;
  if (!parseJointCommand(cmd, idx, angle, duration_ms)) {
    Serial.print("ERR: unknown command '");
    Serial.print(cmd);
    Serial.println("'. Valid: home, status, <joint>-<angle>[-<duration>], pose-...;<duration>");
    signalDone();
    return;
  }

  if (duration_ms > 0) {
    moveJointTrajectory(idx, angle, duration_ms);
  } else {
    moveJointInstant(idx, angle);
    signalDone();
  }
}

void enqueueCommand(const String &cmd) {
  if (queueLen >= MAX_QUEUE) {
    Serial.println("ERR: command queue full");
    return;
  }
  cmdQueue[queueLen++] = cmd;
}

void handleCommands(String input) {
  input.trim();
  if (input.length() == 0) {
    return;
  }

  if (input.startsWith("pose-")) {
    enqueueCommand(input);
  } else {
    int start = 0;
    while (start < (int)input.length()) {
      int semi = input.indexOf(';', start);
      String token = (semi == -1) ? input.substring(start) : input.substring(start, semi);
      token.trim();
      if (token.length() > 0) {
        enqueueCommand(token);
      }
      if (semi == -1) break;
      start = semi + 1;
    }
  }

  if (!commandBusy && queueLen > 0) {
    dispatchCommand(cmdQueue[0]);
  }
}

void updateTrajectory() {
  if (!activeTraj.active) {
    return;
  }

  unsigned long elapsed = millis() - activeTraj.t0;
  float u = (float)elapsed / (float)activeTraj.duration_ms;
  if (u > 1.0f) {
    u = 1.0f;
  }

  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (activeTraj.mask & (1 << i)) {
      int angle = activeTraj.start[i] +
                  (int)((activeTraj.target[i] - activeTraj.start[i]) * u);
      writeJoint(i, angle);
    }
  }

  if (u >= 1.0f) {
    activeTraj.active = false;
    signalDone();
  }
}

// ── Setup ──────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  base_x.attach(PIN_BASE_X);
  base_y.attach(PIN_BASE_Y);
  shoulder.attach(PIN_SHOULDER);
  wrist.attach(PIN_WRIST);
  gripper.attach(PIN_GRIPPER);

  goHomeInstant();
}

// ── Loop ───────────────────────────────────────────
String inputBuffer = "";

void loop() {
  updateTrajectory();

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

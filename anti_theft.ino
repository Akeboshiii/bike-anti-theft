// Anti-theft MVP for Arduino UNO (Updated)
// Components: ADXL345 (I2C optional), GY-NEO6MV2 GPS (NEO-6M family), HC-05 BT, Servo, Buzzer, Vibration sensor
// Libraries: TinyGPSPlus, SoftwareSerial, Servo
// Wiring summary:
//   GY-NEO6MV2: GPS TX -> Arduino pin 4 (SoftwareSerial RX), GPS RX -> Arduino pin 3 (SoftwareSerial TX), baud 9600
//   HC-05 (BT): HC-05 TX -> Arduino pin 10 (SoftwareSerial RX), HC-05 RX -> Arduino pin 11 (SoftwareSerial TX), baud 9600
//   Servo signal: pin 9
//   Buzzer (active): pin 8
//   LED status: pin 7
//   Vibration sensor (digital): pin 5

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// Pins
const int servoPin = 9;
const int buzzerPin = 8;
const int ledPin = 7;
const int vibrationPin = 5;

// SoftwareSerial instances
SoftwareSerial gpsSerial(4, 3);   // RX=4 (GPS TX), TX=3 (GPS RX)  <- GY-NEO6MV2
SoftwareSerial btSerial(10, 11);  // RX=10 (HC-05 TX), TX=11 (HC-05 RX)

TinyGPSPlus gps;
Servo brakeServo;

// Timing & intervals
const unsigned long alarmDurationMs = 30000;       // buzzer + brake duration (ms)
const unsigned long locationSendInterval = 10000;  // ms between periodic updates
const unsigned long motionDebounceMs = 500;        // debounce for vibration

// State
bool armed = false;
bool alarmActive = false;
unsigned long alarmStart = 0;
unsigned long lastLocationSend = 0;
unsigned long lastMotionTime = 0;

// GPS fallback / caching
double lastLat = 0.0;
double lastLon = 0.0;
bool hasFixEver = false; // if true, preserve lastLat/lastLon forever

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(vibrationPin, INPUT);

  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200);
  gpsSerial.begin(9600); // GY-NEO6MV2 default
  btSerial.begin(9600);  // HC-05 default

  brakeServo.attach(servoPin);
  releaseBrake();

  // quick servo reset pulse
  brakeServo.write(0);
  delay(200);
  brakeServo.write(0);

  sendBTState("DISARMED", true);
  Serial.println("Anti-theft system ready.");
}

// Main loop: prioritize GPS reading, then BT, then sensors & state
void loop() {
  // 1) Read GPS (priority)
  gpsSerial.listen();
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    // if a valid GPS fix appears, cache it
    if (gps.location.isValid()) {
      lastLat = gps.location.lat();
      lastLon = gps.location.lng();
      hasFixEver = true;
    }
  }

  // 2) Read BT commands (non-blocking)
  btSerial.listen();
  if (btSerial.available()) {
    String cmd = btSerial.readStringUntil('\n');
    cmd.trim();
    handleBTCommand(cmd);
  }

  // 3) Check motion (debounced)
  if (armed && !alarmActive) {
    int motion = digitalRead(vibrationPin);
    if (motion == HIGH && (millis() - lastMotionTime > motionDebounceMs)) {
      lastMotionTime = millis();
      triggerAlarm();
    }
  }

  // 4) Alarm timeout handling
  if (alarmActive) {
    unsigned long now = millis();
    if (now - alarmStart >= alarmDurationMs) {
      stopAlarm();
    }
  }

  // 5) Periodic location/state updates (rate-limited)
  unsigned long now = millis();
  if (armed && (now - lastLocationSend >= locationSendInterval)) {
    sendBTState("ARMED", false);
    lastLocationSend = now;
  }

  // small loop delay to reduce CPU hogging
  delay(50);
}

// Send status over BT (and mirror to Serial). If force==true, bypass rate limit.
void sendBTState(const String &status, bool force) {
  unsigned long now = millis();
  if (!force && (now - lastLocationSend < locationSendInterval)) return;

  String latS = "0.000000";
  String lonS = "0.000000";

  if (gps.location.isValid()) {
    latS = String(gps.location.lat(), 6);
    lonS = String(gps.location.lng(), 6);
  } else if (hasFixEver) {
    // use cached last known forever (Option 1)
    latS = String(lastLat, 6);
    lonS = String(lastLon, 6);
  } else {
    // No fix ever — Mode C: send 0,0
    latS = "0.000000";
    lonS = "0.000000";
  }

  String msg = status + "," + latS + "," + lonS + "\n";
  btSerial.listen();
  btSerial.print(msg);
  Serial.print("Sent: ");
  Serial.print(msg);

  lastLocationSend = now;
}

void triggerAlarm() {
  alarmActive = true;
  alarmStart = millis();
  engageBrake();
  soundBuzzer(true);
  digitalWrite(ledPin, HIGH);
  sendBTState("STOLEN", true); // force immediate STOLEN broadcast
  Serial.println("ALARM TRIGGERED: STOLEN");
}

void stopAlarm() {
  alarmActive = false;
  soundBuzzer(false);
  releaseBrake();
  digitalWrite(ledPin, LOW);
  sendBTState(armed ? "ARMED" : "DISARMED", true);
  Serial.println("Alarm stopped.");
}

void engageBrake() {
  brakeServo.write(80); // adjust angle for locked position (tune for your servo)
}

void releaseBrake() {
  brakeServo.write(0); // unlocked
}

void soundBuzzer(bool on) {
  digitalWrite(buzzerPin, on ? HIGH : LOW);
}

// Handle simple BT commands: ARM, DISARM, STATUS (returns current)
void handleBTCommand(const String &cmdIn) {
  if (cmdIn.length() == 0) return;
  String cmd = cmdIn;
  cmd.toUpperCase();
  cmd.trim();

  if (cmd == "ARM") {
    armed = true;
    sendBTState("ARMED", true);
    Serial.println("ARMED via BT");
  } else if (cmd == "DISARM") {
    armed = false;
    alarmActive = false;
    soundBuzzer(false);
    releaseBrake();
    sendBTState("DISARMED", true);
    Serial.println("DISARMED via BT");
  } else if (cmd == "STATUS") {
    sendBTState(armed ? "ARMED" : "DISARMED", true);
    Serial.println("STATUS requested via BT");
  } else {
    // Unknown command - echo back
    btSerial.listen();
    btSerial.print("UNKNOWN_CMD\n");
    Serial.print("Unknown BT command: "); Serial.println(cmdIn);
  }
}

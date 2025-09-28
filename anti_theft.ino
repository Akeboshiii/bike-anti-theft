// Anti-theft MVP for Arduino UNO (Analog vibration on A0)
// HC-05 on hardware Serial (pins 0/1). GPS on SoftwareSerial (4=RX,3=TX).
// Vibration sensor: A0 (analog). Trigger condition: analogRead(A0) >= 100

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// Pins
const int servoPin = 9;
const int buzzerPin = 8;
const int ledPin = 7;
const int vibrationPin = A0; // analog input
const int vibrationThreshold = 100; // >= triggers stolen

// GPS on SoftwareSerial
SoftwareSerial gpsSerial(4, 3);   // RX=4 (GPS TX), TX=3 (GPS RX)

TinyGPSPlus gps;
Servo brakeServo;

// Timing
const unsigned long alarmDurationMs = 30000;
const unsigned long locationSendInterval = 10000;
const unsigned long motionDebounceMs = 500;

// State
bool armed = false;
bool alarmActive = false;
unsigned long alarmStart = 0;
unsigned long lastLocationSend = 0;
unsigned long lastMotionTime = 0;

// GPS fallback
double lastLat = 0.0;
double lastLon = 0.0;
bool hasFixEver = false;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  // analog pins don't require pinMode but safe to leave as-is

  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  Serial.begin(9600);   // HC-05 on hardware Serial
  gpsSerial.begin(9600); // GY-NEO6MV2

  brakeServo.attach(servoPin);
  releaseBrake();

  // servo reset
  brakeServo.write(0);
  delay(200);
  brakeServo.write(0);

  sendBTState("DISARMED", true);
  Serial.println("Anti-theft system ready (Analog vibration A0).");
}

void loop() {
  // 1) GPS reads (priority)
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
      lastLat = gps.location.lat();
      lastLon = gps.location.lng();
      hasFixEver = true;
    }
  }

  // 2) BT commands via Serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleBTCommand(cmd);
  }

  // 3) Motion check using analog read (debounced)
  if (armed && !alarmActive) {
    int motionVal = analogRead(vibrationPin); // 0..1023
    if (motionVal >= vibrationThreshold && (millis() - lastMotionTime > motionDebounceMs)) {
      lastMotionTime = millis();
      triggerAlarm();
    }
  }

  // 4) Alarm timeout
  if (alarmActive && (millis() - alarmStart >= alarmDurationMs)) {
    stopAlarm();
  }

  // 5) Periodic state/location updates
  if (armed && (millis() - lastLocationSend >= locationSendInterval)) {
    sendBTState("ARMED", false);
  }

  delay(50);
}

void sendBTState(const String &status, bool force) {
  unsigned long now = millis();
  if (!force && (now - lastLocationSend < locationSendInterval)) return;

  String latS = "0.000000", lonS = "0.000000";
  if (gps.location.isValid()) {
    latS = String(gps.location.lat(), 6);
    lonS = String(gps.location.lng(), 6);
  } else if (hasFixEver) {
    latS = String(lastLat, 6);
    lonS = String(lastLon, 6);
  }

  String msg = status + "," + latS + "," + lonS + "\n";
  Serial.print(msg); // go to HC-05
  lastLocationSend = now;
}

void triggerAlarm() {
  alarmActive = true;
  alarmStart = millis();
  engageBrake();
  soundBuzzer(true);
  digitalWrite(ledPin, HIGH);
  sendBTState("STOLEN", true);
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

void engageBrake() { brakeServo.write(80); }
void releaseBrake() { brakeServo.write(0); }
void soundBuzzer(bool on) { digitalWrite(buzzerPin, on ? HIGH : LOW); }

void handleBTCommand(const String &cmdIn) {
  if (cmdIn.length() == 0) return;
  String cmd = cmdIn; cmd.toUpperCase(); cmd.trim();

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
    Serial.print("UNKNOWN_CMD\n");
  }
}

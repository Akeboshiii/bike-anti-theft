// Anti-theft MVP for Arduino UNO (BT via Serial)
// Components: ADXL345 (I2C optional), GY-NEO6MV2 GPS, HC-05 BT, Servo, Buzzer, Vibration sensor
// Libraries: TinyGPSPlus, SoftwareSerial, Servo
// GPS on SoftwareSerial (pins 4=RX, 3=TX)
// HC-05 BT on hardware Serial (pins 0=RX, 1=TX) at 9600 baud
// NOTE: disconnect HC-05 while uploading code via USB

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// Pins
const int servoPin = 9;
const int buzzerPin = 8;
const int ledPin = 7;
const int vibrationPin = 5;

// GPS stays on SoftwareSerial
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
  pinMode(vibrationPin, INPUT);

  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  Serial.begin(9600);   // HC-05 default baud
  gpsSerial.begin(9600);

  brakeServo.attach(servoPin);
  releaseBrake();

  // reset servo
  brakeServo.write(0); delay(200); brakeServo.write(0);

  sendBTState("DISARMED", true);
  Serial.println("Anti-theft system ready (BT on Serial).");
}

void loop() {
  // 1) GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
      lastLat = gps.location.lat();
      lastLon = gps.location.lng();
      hasFixEver = true;
    }
  }

  // 2) BT commands (via Serial)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleBTCommand(cmd);
  }

  // 3) Motion
  if (armed && !alarmActive) {
    if (digitalRead(vibrationPin) == HIGH && (millis() - lastMotionTime > motionDebounceMs)) {
      lastMotionTime = millis();
      triggerAlarm();
    }
  }

  // 4) Alarm timeout
  if (alarmActive && (millis() - alarmStart >= alarmDurationMs)) {
    stopAlarm();
  }

  // 5) Periodic state updates
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
  Serial.print(msg);  // goes to HC-05 now
  lastLocationSend = now;
}

void triggerAlarm() {
  alarmActive = true;
  alarmStart = millis();
  engageBrake();
  soundBuzzer(true);
  digitalWrite(ledPin, HIGH);
  sendBTState("STOLEN", true);
}

void stopAlarm() {
  alarmActive = false;
  soundBuzzer(false);
  releaseBrake();
  digitalWrite(ledPin, LOW);
  sendBTState(armed ? "ARMED" : "DISARMED", true);
}

void engageBrake() { brakeServo.write(80); }
void releaseBrake() { brakeServo.write(0); }
void soundBuzzer(bool on) { digitalWrite(buzzerPin, on ? HIGH : LOW); }

void handleBTCommand(const String &cmdIn) {
  if (cmdIn.length() == 0) return;
  String cmd = cmdIn; cmd.toUpperCase();

  if (cmd == "ARM") {
    armed = true;
    sendBTState("ARMED", true);
  } else if (cmd == "DISARM") {
    armed = false;
    alarmActive = false;
    soundBuzzer(false);
    releaseBrake();
    sendBTState("DISARMED", true);
  } else if (cmd == "STATUS") {
    sendBTState(armed ? "ARMED" : "DISARMED", true);
  } else {
    Serial.print("UNKNOWN_CMD\n");
  }
}

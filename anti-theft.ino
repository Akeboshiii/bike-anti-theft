// Anti-theft MVP for Arduino UNO
// Components: ADXL345 (I2C), NEO-6M GPS, HC-05 BT, Servo (brake), Buzzer
// Libraries needed: Adafruit_ADXL345_U, TinyGPSPlus, Servo

#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// --- Pins ---
const int servoPin = 9;
const int buzzerPin = 8;
const int ledPin = 7;

// SoftwareSerial: gpsSerial (RX=4, TX=3), btSerial (RX=10, TX=11)
SoftwareSerial gpsSerial(4, 3);   // GPS TX -> pin 4
SoftwareSerial btSerial(10, 11);  // HC-05 TX -> pin 10

// --- Sensors & devices ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
TinyGPSPlus gps;
Servo brakeServo;

// --- Configurable params ---
const float accelThreshold = 1.5;       // g threshold to trigger alarm
const unsigned long alarmDurationMs = 30000; // buzzer duration
const unsigned long locationSendInterval = 10000; // ms between updates

// --- State ---
bool armed = false;
bool alarmActive = false;
unsigned long alarmStart = 0;
unsigned long lastLocationSend = 0;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200); 
  gpsSerial.begin(9600);
  btSerial.begin(9600);

  // Init accelerometer
  if (!accel.begin()) {
    for (;;) { // blink LED forever if ADXL fails
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
    }
  }
  accel.setRange(ADXL345_RANGE_16_G);

  brakeServo.attach(servoPin);
  releaseBrake();

  // Initial state
  sendBTState("DISARMED");
}

void loop() {
  // 1) Read BT commands
  btSerial.listen();
  if (btSerial.available()) {
    String cmd = btSerial.readStringUntil('\n');
    cmd.trim();
    handleBTCommand(cmd);
  }

  // 2) Update GPS
  gpsSerial.listen();
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // 3) Check motion
  if (armed && !alarmActive) {
    sensors_event_t event;
    accel.getEvent(&event);
    float gx = event.acceleration.x / 9.80665;
    float gy = event.acceleration.y / 9.80665;
    float gz = event.acceleration.z / 9.80665;
    float mag = sqrt(gx*gx + gy*gy + gz*gz);

    float delta = fabs(mag - 1.0);
    if (delta >= accelThreshold) {
      triggerAlarm();
    }
  }

  // 4) Alarm timeout
  if (alarmActive) {
    unsigned long now = millis();
    if (now - alarmStart >= alarmDurationMs) {
      stopAlarm();
    }
  }

  // 5) Periodic location updates
  unsigned long now = millis();
  if (armed && (now - lastLocationSend >= locationSendInterval)) {
    sendBTState(armed ? "ARMED" : "DISARMED");
    lastLocationSend = now;
  }

  delay(50);
}

// --- Core functions ---
void sendBTState(const String &status) {
  String lat = "NA", lon = "NA";
  if (gps.location.isValid()) {
    lat = String(gps.location.lat(), 6);
    lon = String(gps.location.lng(), 6);
  }
  String msg = status + "," + lat + "," + lon + "\n";
  btSerial.listen();
  btSerial.print(msg);
  Serial.print("Sent: "); Serial.print(msg);
}

void triggerAlarm() {
  alarmActive = true;
  alarmStart = millis();
  engageBrake();
  soundBuzzer(true);
  digitalWrite(ledPin, HIGH);
  sendBTState("STOLEN");
}

void stopAlarm() {
  alarmActive = false;
  soundBuzzer(false);
  releaseBrake();
  digitalWrite(ledPin, LOW);
  sendBTState(armed ? "ARMED" : "DISARMED");
}

void engageBrake() {
  brakeServo.write(80); // adjust angle for locked position
}

void releaseBrake() {
  brakeServo.write(0); // adjust angle for unlocked
}

void soundBuzzer(bool on) {
  digitalWrite(buzzerPin, on ? HIGH : LOW);
}

void handleBTCommand(const String &cmdIn) {
  if (cmdIn.length() == 0) return;
  String cmd = cmdIn;
  cmd.toUpperCase();
  cmd.trim();

  if (cmd == "ARM") {
    armed = true;
    sendBTState("ARMED");
    Serial.println("ARMED");
  } else if (cmd == "DISARM") {
    armed = false;
    alarmActive = false;
    soundBuzzer(false);
    releaseBrake();
    sendBTState("DISARMED");
    Serial.println("DISARMED");
  }
}

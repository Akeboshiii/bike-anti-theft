// Debug Mode for Anti-Theft Sensors
// Tests: ADXL345 (via I2C), Vibration Pin, GPS (NEO-6M), Servo, Buzzer, LED, Bluetooth

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// Pin Definitions
const int servoPin = 9;
const int buzzerPin = 8;
const int ledPin = 7;
const int vibrationPin = 5;

SoftwareSerial gpsSerial(4, 3);   // GPS TX -> pin 4
SoftwareSerial btSerial(10, 11);  // HC-05 TX -> pin 10

TinyGPSPlus gps;
Servo brakeServo;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  btSerial.begin(9600);

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(vibrationPin, INPUT);

  brakeServo.attach(servoPin);
  brakeServo.write(0); // Initial position

  Serial.println("=== DEBUG MODE STARTED ===");
  btSerial.println("=== DEBUG MODE STARTED ===");
}

void loop() {
  debugVibration();
  debugGPS();
  debugServo();
  debugBuzzer();
  debugLED();
  debugBluetoothEcho();

  delay(500);
}

// --- Debug Functions ---

void debugVibration() {
  int v = digitalRead(vibrationPin);
  Serial.print("[VIBRATION] "); Serial.println(v);
}

void debugGPS() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isValid()) {
    Serial.print("[GPS] LAT: "); Serial.print(gps.location.lat(), 6);
    Serial.print("  LON: "); Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("[GPS] Waiting for signal...");
  }
}

void debugServo() {
  static bool toggle = false;
  brakeServo.write(toggle ? 80 : 0);
  Serial.print("[SERVO] Position: "); Serial.println(toggle ? 80 : 0);
  toggle = !toggle;
}

void debugBuzzer() {
  digitalWrite(buzzerPin, HIGH);
  Serial.println("[BUZZER] ON");
  delay(200);
  digitalWrite(buzzerPin, LOW);
  Serial.println("[BUZZER] OFF");
}

void debugLED() {
  digitalWrite(ledPin, HIGH);
  Serial.println("[LED] ON");
  delay(200);
  digitalWrite(ledPin, LOW);
  Serial.println("[LED] OFF");
}

void debugBluetoothEcho() {
  if (btSerial.available()) {
    String msg = btSerial.readStringUntil('\n');
    Serial.print("[BT RECEIVED] "); Serial.println(msg);
    btSerial.print("[ECHO] "); btSerial.println(msg);
  }
}

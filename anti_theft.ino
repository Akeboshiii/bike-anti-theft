// Anti-theft MVP for Arduino UNO
// Components: ADXL345 (I2C), NEO-6M GPS, HC-05 BT, Servo (brake), Buzzer
// Libraries needed: Adafruit_ADXL345_U, TinyGPSPlus, Servo
//
//ADXL345 (I²C): SDA → A4, SCL → A5, VCC → 3.3V (or 5V per module), GND → GND (NOT USED)
//Vibration Sensor: OUT → Arduino pin 5, VCC → 3.3v  
//NEO-6M GPS (TTL): GPS TX → Arduino pin 4, GPS RX → Arduino pin 3 (SoftwareSerial)
//HC-05 BT module: HC-05 TX → Arduino pin 11, HC-05 RX → Arduino pin 10 (SoftwareSerial)
//Servo (brake): signal → pin 9, VCC → 5V (external if servo draws >500mA), GND common
//Buzzer (active): → pin 8, other side → GND
//Optional: LED status → pin 7 → GND via resistor

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

const int servoPin = 9;
const int buzzerPin = 8;
const int ledPin = 7;
const int vibrationPin = 5;

// SoftwareSerial: gpsSerial (RX=4, TX=3), btSerial (RX=10, TX=11)
SoftwareSerial gpsSerial(4, 3);   // GPS TX -> pin 4
SoftwareSerial btSerial(10, 11);  // HC-05 TX -> pin 10

TinyGPSPlus gps;
Servo brakeServo;

const unsigned long alarmDurationMs = 30000; // buzzer duration
const unsigned long locationSendInterval = 10000; // ms between updates

bool armed = false;
bool alarmActive = false;
unsigned long alarmStart = 0;
unsigned long lastLocationSend = 0;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(vibrationPin, INPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  Serial.begin(115200); 
  gpsSerial.begin(9600);
  btSerial.begin(9600);

  brakeServo.attach(servoPin);
  releaseBrake();

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
    if (digitalRead(vibrationPin) > 0) {
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

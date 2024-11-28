#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <HX711.h>

#define CRANK_RADIUS 0.1725
#define DATA_PIN 3
#define CLK_PIN 2

HX711 scale;
BLEService CyclePowerService("1818");
BLECharacteristic CyclePowerFeature("2A65", BLERead, 4);
BLECharacteristic CyclePowerMeasurement("2A63", BLERead | BLENotify, 8);
BLECharacteristic CyclePowerSensorLocation("2A5D", BLERead, 1);

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

short power = 0;
unsigned short rev = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20;
byte sensorlocation = 0x0D;

long previousMillis = 0;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");

  scale.begin(DATA_PIN, CLK_PIN);
  scale.set_offset(398895);
  scale.set_scale(4.538056);
  scale.tare();


  // Start BLE module
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  Serial.println("BLE initialized.");

  // Set device name and local name
  BLE.setDeviceName("ArduinoPowerMeter");
  BLE.setLocalName("ArduinoPowerMeter");
  BLE.setAdvertisedService(CyclePowerService);
  CyclePowerService.addCharacteristic(CyclePowerFeature);
  CyclePowerService.addCharacteristic(CyclePowerMeasurement);
  CyclePowerService.addCharacteristic(CyclePowerSensorLocation);

  BLE.addService(CyclePowerService);
  BLE.advertise();

  Serial.println("BLE service advertised. Waiting for connections...");
}

void loop() {
  // Poll for BLE events
  BLE.poll();

  long currentMillis = millis();
  // if 250ms have passed, check send the data:
  if (currentMillis - previousMillis >= 500) {
    timestamp = (unsigned short) currentMillis;

    float x, y, z;
    IMU.readGyroscope(x, y, z);

    float rotz = fabs(z);
    // Magic number here, but less than 45 dps is less than 1 crank rotation 
    // in 8 seconds (7.5 RPM), just assume it's noise.
    if (rotz < 45) rotz = 0.f;
    float velo = (rotz * PI * CRANK_RADIUS) / 180.f; // Velocity of crank in m/s
    float f = fabs(scale.get_units(10) / 102.f); // Force on the pedal in N (grams -> newtons @ 9.81/1000 -> 101.9367)

    power = short(2 * f * velo); // P = 2 * N * m/s
    rev = short(rotz / 60.f); // deg/min (60s) / 360 deg = deg/s / 60

    // Update the characteristic value
    bleBuffer[0] = flags & 0xff;
    bleBuffer[1] = (flags >> 8) & 0xff;
    bleBuffer[2] = power & 0xff;
    bleBuffer[3] = (power >> 8) & 0xff;
    bleBuffer[4] = rev & 0xff;
    bleBuffer[5] = (rev >> 8) & 0xff;
    bleBuffer[6] = timestamp & 0xff;
    bleBuffer[7] = (timestamp >> 8) & 0xff;

    slBuffer[0] = sensorlocation & 0xff;

    fBuffer[0] = 0x00;
    fBuffer[1] = 0x00;
    fBuffer[2] = 0x00;
    fBuffer[3] = 0x08;
            
    CyclePowerFeature.writeValue(fBuffer, 4);
    CyclePowerMeasurement.writeValue(bleBuffer, 8);
    CyclePowerSensorLocation.writeValue(slBuffer, 1);

    previousMillis = currentMillis;

    // DEBUG
    
    Serial.print("Rot:");
    Serial.println(rotz);
    Serial.print("Speed:");
    Serial.println(velo);
    Serial.print("Force:");
    Serial.println(f);
    Serial.print("Power:");
    Serial.println(power);
    Serial.print("Rev:");
    Serial.println(rev);
  }
}
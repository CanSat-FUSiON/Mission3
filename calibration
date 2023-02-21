#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.println("Could not find a valid BNO055 sensor, check wiring!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true); // 外部クリスタルを使用
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  // キャリブレーションが必要な場合は、キャリブレーションを行う
  if (!bno.isFullyCalibrated()) {
    Serial.println("Performing IMU calibration...");
    while (!bno.isFullyCalibrated()) {
      delay(1000);
    }
    Serial.println("IMU calibration complete");
  }

  // 地磁気キャリブレーションが必要な場合は、キャリブレーションを行う
  if (!bno.isMagCalibrated()) {
    Serial.println("Performing magnetometer calibration...");
    while (!bno.isMagCalibrated()) {
      bno.getEvent(&event);
      delay(100);
    }
    Serial.println("Magnetometer calibration complete");
  }

  Serial.print("Orientation: ");
  Serial.print(event.orientation.x);
  Serial.print(" ");
  Serial.print(event.orientation.y);
  Serial.print(" ");
  Serial.println(event.orientation.z);
  delay(100);
}

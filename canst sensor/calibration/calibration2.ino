#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (!bno.begin()) {
    Serial.println("BNO055 not found");
    while (1);
  }
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  if (event.type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orientation: ");
    Serial.print(event.orientation.x);
    Serial.print(", ");
    Serial.print(event.orientation.y);
    Serial.print(", ");
    Serial.println(event.orientation.z);
  }
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("Calibration: ");
  Serial.print(system, DEC);
  Serial.print(", ");
  Serial.print(gyro, DEC);
  Serial.print(", ");
  Serial.print(accel, DEC);
  Serial.print(", ");
  Serial.println(mag, DEC);

  if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
    Serial.println("Calibration Complete");
  } else {
    Serial.println("Calibration Incomplete");
  }
  delay(500);
}

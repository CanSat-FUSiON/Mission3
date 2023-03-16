#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <MsTimer2.h>

// PWM出力端子設定
#define pwm1 14
#define dir1 27
#define pwm2 26
#define dir2 25
#define STACK_THRESHOLD 0.02

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH1 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

//MD2個使うからチャンネルも二個必要かな？1こ出善さげな感じする
#define CH2 0        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）



/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  // PWM出力に使用する端子を出力設定
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);

  // PWM初期設定
  ledcSetup(CH1, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcSetup(CH2, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir1, CH1);
  ledcAttachPin(dir2, CH2);

  digitalWrite(pwm1, HIGH);
  digitalWrite(pwm2, HIGH);

  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);

  /*
    if (accel < STACK_THRESHOLD) {
      delay(2000); 　 //けっきょくこれGPSRUNフェーズに入ってからだから旋回するときとか以外止まらないはず
      if (accel < STACK_THRESHOLD - 0.1) {
        ledcWrite(CH1, 4096); //後退→右に旋回→直進
        ledcWrite(CH2, 4096);
        delay(100);
        ledcWrite(CH1, 0);
        ledcWrite(CH2, 2048);
        delay(100);
        ledcWrite(CH1, 0);
        ledcWrite(CH2, 0);
        delay(100);
      }else{
        delay(100);
      }
    }
  */



  if (accel < STACK_THRESHOLD) {
    delay(2000); //けっきょくこれGPSRUNフェーズに入ってからだから旋回するときとか以外止まらないはず
    if (accel < STACK_THRESHOLD) {
      Serial.println("Robot is stacked!");
      ledcWrite(CH1, 4096); //後退→右に旋回→直進
      ledcWrite(CH2, 4096);
      delay(100);
      ledcWrite(CH1, 0);
      ledcWrite(CH2, 2048);
      delay(100);
      ledcWrite(CH1, 0);
      ledcWrite(CH2, 0);
      delay(100);
    }
  }
}


  void printEvent(sensors_event_t* event) {
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
      Serial.print("Accl:");
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION) {
      Serial.print("Orient:");
      x = event->orientation.x;
      y = event->orientation.y;
      z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
      Serial.print("Mag:");
      x = event->magnetic.x;
      y = event->magnetic.y;
      z = event->magnetic.z;
    }
    else if (event->type == SENSOR_TYPE_GYROSCOPE) {
      Serial.print("Gyro:");
      x = event->gyro.x;
      y = event->gyro.y;
      z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
      Serial.print("Rot:");
      x = event->gyro.x;
      y = event->gyro.y;
      z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
      Serial.print("Linear:");
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_GRAVITY) {
      Serial.print("Gravity:");
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
    }
    else {
      Serial.print("Unk:");
    }

    Serial.print("\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
  }

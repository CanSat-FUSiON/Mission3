// Pins
#define ENCA 35
#define ENCB 34

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;

float vFilt = 0;
float vPrev = 0;

float kp = 15;
float ki = 100;
float eintegral = 0;
int pwr;

// PWM出力端子設定
#define pwm 14
#define dir 27


// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

// 変数設定
float duty;         // Duty比
float resolution;   // 分解能

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // PWM出力に使用する端子を出力設定
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  // PWM初期設定
  ledcSetup(CH, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir, CH);

  digitalWrite(pwm, HIGH);
}

void loop() {

  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v = velocity / 244.8 * 60.0;

  // Low-pass filter (25 Hz cutoff)
  vFilt = 0.854 * vFilt + 0.0728 * v + 0.0728 * vPrev;
  vPrev = v;

  // Set a target
  //float vt = 400 * (sin(currT / 1e6) > 0);
  float vt = 400*sin(currT / 1e6);

  // Compute the control signal u
  float e = vt - vFilt;
  eintegral = eintegral + e * deltaT;

  float u = kp * e + ki * eintegral;

  // Set the motor speed and direction
  if (abs(u) <= 2048) {
    pwr = u + 2048;
  } else if (u > 2048) {
    pwr = 4096;
  } else {
    pwr = 0;
  }
  ledcWrite(CH, pwr);

  Serial.print("Variable_1:");
  Serial.print(vt);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(vFilt);
}


void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment backward
    increment = -1;
  }
  else {
    // Otherwise, increment forward
    increment = 1;
  }
  pos_i = pos_i + increment;
}

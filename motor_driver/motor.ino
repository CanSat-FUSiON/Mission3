const int motorA[3] = {13, 4, 25};  // AIN1, AIN2, PWMA
const int motorB[3] = {14, 27, 26}; // BIN1, BIN2, PWMB

const int CHANNEL_A = 0;
const int CHANNEL_B = 1;

const int LEDC_TIMER_BIT = 8;
const int LEDC_BASE_FREQ = 490;

void setup() {
  for(int i = 0; i < 3; i++){
    pinMode(motorA[i], OUTPUT);
    pinMode(motorB[i], OUTPUT);
  }

  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

  ledcAttachPin(motorA[2], CHANNEL_A);
  ledcAttachPin(motorB[2], CHANNEL_B);
}

void loop() {
  forward(80);
  delay(1000);

  stop();
  delay(1000);

  back(80);
  delay(1000);

  stop();
  delay(1000);
}

/** 前進 */
void forward(int pwm) {
  // 左モータ（CCW，反時計回り）
  digitalWrite(motorA[1], LOW);
  digitalWrite(motorA[0], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CW，時計回り）
  digitalWrite(motorB[1], LOW);
  digitalWrite(motorB[0], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 後退 */
void back(int pwm) {
  // 左モータ（CW，時計回り）
  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CCW，反時計回り）
  digitalWrite(motorB[0], LOW);
  digitalWrite(motorB[1], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/** 停止 */
void stop() {
  // 左モータ停止
  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], LOW);
  ledcWrite(CHANNEL_A, HIGH);

  // 右モータ停止
  digitalWrite(motorB[0], LOW);
  digitalWrite(motorB[1], LOW);
  ledcWrite(CHANNEL_B, HIGH);
}

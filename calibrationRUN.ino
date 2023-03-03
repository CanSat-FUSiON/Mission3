// PWM出力端子設定
#define pwm1 14
#define dir1 27
#define pwm2 26
#define dir2 25

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH1 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

#define CH2 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

// 初期設定 -----------------------------------------
void setup() {
  // PWM出力に使用する端子を出力設定
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);

  // PWM初期設定
  ledcSetup(CH1, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir1, CH1);
  ledcSetup(CH2, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir2, CH1);

  digitalWrite(pwm1, HIGH);
  digitalWrite(pwm2, HIGH);
}
// メイン -------------------------------------------
void loop() {
  for (int i = 1000; i <= 2596; i++) {
    ledcWrite(CH1, i); //正回転 --> active ブレーキ --> 逆回転
    ledcWrite(CH2, i);
    delay(1000);
  }
  ledcWrite(CH1, 1000);
  ledcWrite(CH2, 1500);
  delay(5000);
  ledcWrite(CH1, 1500);
  ledcWrite(CH2, 1000);
  delay(5000);
  delay(5000);

}

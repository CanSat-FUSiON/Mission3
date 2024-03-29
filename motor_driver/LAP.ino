// PWM出力端子設定
#define pwm 14
#define dir 27


// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

// 初期設定 -----------------------------------------
void setup() {
  // PWM出力に使用する端子を出力設定
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  // PWM初期設定
  ledcSetup(CH, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir, CH);

  digitalWrite(pwm, HIGH);
}
// メイン -------------------------------------------
void loop() {
  ledcWrite(CH, 0); //フル正回転
  delay(5000);
  ledcWrite(CH, 4096); //フル逆回転
  delay(5000);
  ledcWrite(CH, 2048); //active ブレーキ
  delay(5000);
  for (int i = 0; i <= 4096; i++) {
    ledcWrite(CH, i); //フル正回転 --> active ブレーキ --> フル逆回転
    delay(10);
  }
  delay(5000);
}



// PWM出力端子設定
#define pwm_2 26     
#define pwm_1 27
#define dir_2 25
#define dir_1 14
 

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 4.9   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

// 変数設定
float ad_val;       // アナログ入力値格納用
float duty;         // Duty比
float resolution;   // 分解能
float battery;      // バッテリー残量表示用
bool state = false; // ボタン操作状態保持用

// 初期設定 -----------------------------------------
void setup() {
  Serial.begin(115200); // シリアル出力初期化（今回は未使用）

  // PWM出力に使用する端子を出力設定
  pinMode(pwm_2, OUTPUT);     
  pinMode(pwm_1, OUTPUT);     
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);

  // PWM初期設定
  ledcSetup(CH, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir_2, CH); // PWMﾁｬﾝﾈﾙ割り当て（端子番号, ﾁｬﾝﾈﾙ）
  ledcAttachPin(dir_1, CH); 

  int reading = 0;
  int output=0;
  int opposite_output=0;
  digitalWrite(pwm_1,HIGH);
  digitalWrite(pwm_2,HIGH);

  // PWM出力実行（以下の3ついずれか1つを有効にして動作確認）
  digitalWrite(pwm_1,LOW);
  digitalWrite(pwm_2,LOW);
  delay(10000);
  ledcWrite(1, 1500);       // チャンネルに指定した周波数、分解能で実行する場合（チャンネル、Duty比）
  delay(5000);
  digitalWrite(pwm_1,LOW);
  digitalWrite(pwm_2,LOW);
}
// メイン -------------------------------------------
void loop() {

}

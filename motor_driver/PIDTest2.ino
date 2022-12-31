// Pins
#define ENCA 35
#define ENCB 34

long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;

float vFilt = 0;

//2 Hz cutoff frequency
float kp = 40; //too low, low rise time, too high, oscillatory
float ki = 30; //too low, steady-state error, too high, overshoot
float kd = 1.5; //too low, oscillatory, too high, chattering

//3 Hz cutoff frequency
//float kp = 40; //too low, low rise time, too high, oscillatory
//float ki = 45; //too low, steady-state error, too high, overshoot
//float kd = 1.2; //too low, oscillatory, too high, chattering

float eintegral = 0;
float eprev = 0;
float dedt = 0;
int pwr;

// PWM出力端子設定
#define pwm 14
#define dir 27

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
LowPass<2> lp(2,1e3,true);

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
  vFilt = lp.filt(v);

  // Set a target
  //float vt = 400 * (sin(currT / 1e6) > 0);
  float vt = 400 * sin(currT / 1e6 * 2);

  // Compute the control signal u
  float e = vt - vFilt;
  dedt = (e-eprev)/(deltaT);
  eintegral = eintegral + e * deltaT;

  float u = kp * e + ki * eintegral + kd * dedt;
  eprev = e;

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

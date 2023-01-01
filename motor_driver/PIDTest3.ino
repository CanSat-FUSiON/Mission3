// Low Pass Filter Class
template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order + 1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order + 1]; // Raw values
    float y[order + 1]; // Filtered values

  public:
    LowPass(float f0, float fs, bool adaptive) {
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.

      omega0 = 6.28318530718 * f0;
      dt = 1.0 / fs;
      adapt = adaptive;
      tn1 = -dt;
      for (int k = 0; k < order + 1; k++) {
        x[k] = 0;
        y[k] = 0;
      }
      setCoef();
    }

    void setCoef() {
      if (adapt) {
        float t = micros() / 1.0e6;
        dt = t - tn1;
        tn1 = t;
      }

      float alpha = omega0 * dt;
      if (order == 1) {
        a[0] = -(alpha - 2.0) / (alpha + 2.0);
        b[0] = alpha / (alpha + 2.0);
        b[1] = alpha / (alpha + 2.0);
      }
      if (order == 2) {
        float alphaSq = alpha * alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
        b[0] = alphaSq / D;
        b[1] = 2 * b[0];
        b[2] = b[0];
        a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
        a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
      }
    }

    float filt(float xn) {
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if (adapt) {
        setCoef(); // Update coefficients if necessary
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for (int k = 0; k < order; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
      }
      y[0] += b[order] * x[order];

      // Save the historical values
      for (int k = order; k > 0; k--) {
        y[k] = y[k - 1];
        x[k] = x[k - 1];
      }

      // Return the filtered value
      return y[0];
    }
};

// Filter instance
LowPass<2> lp(3, 1e3, true);

#define N_MOTORS 1

// Pins
const int encA[] = {35, 39};
const int encB[] = {34, 36};
const int pwm[] = {14, 26};
const int dir[] = {27, 25};

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）
#define PPR 244.8

// Globals
long currT, prevT = 0;
int pos[N_MOTORS], power;
float deltaT, target[N_MOTORS], rpm[N_MOTORS], rpmFilt[N_MOTORS],  e[N_MOTORS],
      eprev[N_MOTORS], dedt[N_MOTORS], eintegral[N_MOTORS], u[N_MOTORS];
int prevPos[] = {0, 0};
volatile int pos_i[] = {0, 0};
float kp[] = {40, 1};
float ki[] = {30, 1};
float kd[] = {1.5, 1};

template <int i>
void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(encB[i]);
  if (b > 0) {
    // If B is high, increment backward
    pos_i[i]--;
  }
  else {
    // Otherwise, increment forward
    pos_i[i]++;
  }
}

void PIDController(int i) {
  e[i] = target[i] - rpmFilt[i];
  dedt[i] = (e[i] - eprev[i]) / deltaT;
  eprev[i] = e[i];
  eintegral[i] = eintegral[i] + e[i] * deltaT;
  u[i] = kp[i] * e[i] + ki[i] * eintegral[i] + kd[i] * dedt[i];

  // Set the motor speed and direction
  if (abs(u[i]) <= 2048) {
    power = u[i] + 2048;
  } else if (u[i] > 2048) {
    power = 4096;
  } else {
    power = 0;
  }
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < N_MOTORS; i++) {
    pinMode(encA[i], INPUT);
    pinMode(encB[i], INPUT);
    pinMode(pwm[i], OUTPUT);
    digitalWrite(pwm[i], HIGH);
    pinMode(dir[i], OUTPUT);
    ledcSetup(i, FREQ, BIT_NUM);
    ledcAttachPin(dir[i], i);
  }

  attachInterrupt(digitalPinToInterrupt(encA[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(encA[1]), readEncoder<1>, RISING);
}

void loop() {
  target[0] = 400 * sin(currT / 1e6 * 16);

  for (int i = 0; i < N_MOTORS; i++) {
    pos[i] = 0;
  }

  // Read the encoders
  noInterrupts(); // disable interrupts temporarily while reading to avoid misreading
  for (int i = 0; i < N_MOTORS; i++) {
    pos[i] = pos_i[i];
  }
  interrupts(); // turn interrupts back on

  // Compute time difference
  currT = micros();
  deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;

  // Loop through the motors
  for (int i = 0; i < N_MOTORS; i++) {
    rpm[i] = ((pos[i] - prevPos[i]) / deltaT) / PPR * 60.0;
    prevPos[i] = pos[i];
    rpmFilt[i] = lp.filt(rpm[i]);
    // evaluate the control signal
    PIDController(i);
    ledcWrite(i, power);
  }

  for (int i = 0; i < N_MOTORS; i++) {
    Serial.print("Variable_1:");
    Serial.print(target[i]);
    Serial.print(",");
    Serial.print("Variable_2:");
    Serial.println(rpmFilt[i]);
  }
}

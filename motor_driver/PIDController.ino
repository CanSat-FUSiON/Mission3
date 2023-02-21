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
LowPass<2> lp0(3, 1e3, true);
LowPass<2> lp1(3, 1e3, true);

// ループ時間が長くなるほど測定ノイズが多くなるため、滑らかな出力のために遮断周波数を下げる必要がある　＋　PIDをチューニング
//LowPass<2> lp0(0.6, 1e3, true);
//LowPass<2> lp1(0.6, 1e3, true);

// Motor settings
#define FREQ 20000
#define BIT_NUM 12
#define PPR 244.8
#define PID_COUNT 3
#define MOTOR_COUNT 2

// Encoder and motor settings
const int encA[MOTOR_COUNT] = {35, 39};
const int encB[MOTOR_COUNT] = {34, 36};
const int pwm[MOTOR_COUNT] = {14, 26};
const int dir[MOTOR_COUNT] = {27, 25};

// Encoder and motor global variables
long currT, prevT = 0;
int pos[PID_COUNT];
float deltaT, power, target[PID_COUNT], rpm[MOTOR_COUNT], measurement[PID_COUNT],  e[PID_COUNT],
      eprev[PID_COUNT], dedt[PID_COUNT], eintegral[PID_COUNT], u[PID_COUNT];
int prevPos[MOTOR_COUNT] = {0, 0};
volatile int pos_i[MOTOR_COUNT] = {0, 0};
float kp[PID_COUNT] = {40, 40, 1};
float ki[PID_COUNT] = {30, 30, 1};
float kd[PID_COUNT] = {1.5, 1.5, 0};

// Interrupt service routine (ISR)
template <int i>
void readEncoder() {
  // Read encoder B when ENCA rises
  bool b = digitalRead(encB[i]);
  if (b > 0) {
    // If B is high, increment backward
    pos_i[i]--;
  }
  else {
    // Otherwise, increment forward
    pos_i[i]++;
  }
}

// PID Controller
// i= 0, 1:
// Motors
// i = 2:
// Heading
float PIDController(int i, int maxVal, int minVal) {
  e[i] = target[i] - measurement[i];
  dedt[i] = (e[i] - eprev[i]) / deltaT;
  eprev[i] = e[i];
  eintegral[i] = eintegral[i] + e[i] * deltaT;

  // Integrator anti-windup
  if (ki[i] * eintegral[i] > maxVal) {
    eintegral[i] = maxVal / ki[i];
  } else if (ki[i] * eintegral[i] < -minVal) {
    eintegral[i] = -minVal / ki[i];
  }

  // Compute the input value of actuator
  u[i] = kp[i] * e[i] + ki[i] * eintegral[i] + kd[i] * dedt[i];

  // Specific conditions
  if (i == 0 | i == 1) {
    if (abs(u[i]) <= 2048) {
      return u[i] + 2048;
    } else if (u[i] > 2048) {
      return 4096;
    } else {
      return 0;
    }
  } else if (i == 2) {
    if (u[i] > maxVal) {
      u[i] = maxVal;
    } else if  (u[i] < minVal) {
      u[i] = minVal;
    }
    return u[i];
  }
}


void setup() {
  Serial.begin(115200);

  // Set up the encoder and motor pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(encA[i], INPUT);
    pinMode(encB[i], INPUT);
    pinMode(pwm[i], OUTPUT);
    digitalWrite(pwm[i], HIGH);
    pinMode(dir[i], OUTPUT);
    ledcSetup(i, FREQ, BIT_NUM);
    ledcAttachPin(dir[i], i);
  }

  // Attach the interrupt pins to encoders
  attachInterrupt(digitalPinToInterrupt(encA[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(encA[1]), readEncoder<1>, RISING);
}

void loop() {

  // Set the target RPM values
  // Target値の設定について：Target値の変化が急だと、応答が振動する可能性があるため、一次ローパスフィルター（一次遅れ系）を入れた方がいい
  target[0] = 400 * sin(currT / 1e6 * 5);
  target[1] = -500 * sin(currT / 1e6 * 2);

  // Initialize the position variable
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pos[i] = 0;
  }

  // Read the encoders
  noInterrupts(); // Disable interrupts temporarily while reading to avoid misreading
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pos[i] = pos_i[i];
  }
  interrupts();

  // Compute the time difference
  currT = micros();
  deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;

  // Compute the RPM
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rpm[i] = ((pos[i] - prevPos[i]) / deltaT) / PPR * 60.0;
    prevPos[i] = pos[i];
  }

  // Filter the RPM using a 2nd order low pass filter
  measurement[0] = lp0.filt(rpm[0]);
  measurement[1] = lp1.filt(rpm[1]);

  // Evaluate the control signal and control the motors
  for (int i = 0; i < MOTOR_COUNT; i++) {
    power = PIDController(i, 2048, -2048);
    ledcWrite(i, power);
  }

  // Print the motors' filtered RPM
  Serial.print("Variable_1:");
  Serial.print(measurement[0]);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(measurement[1]);
}

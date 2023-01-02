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
LowPass<2> lp(0.7, 1e3, true);

// Ultrasonic sensor library
#include "NewPing.h"

// Ultrasonic sensor pins
#define TRIGGER_PIN  4
#define ECHO_PIN     5
#define MAX_DISTANCE 400

// Constructor for ultrasonic sensor library
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Globals for ultrasonic sensor
int temp = 27; // Celsius
float soundsp;
float duration, distance, kaldist, lpdist;
int iterations = 3;

// Kalman filter function
double kalman(double U) {
  static const float R = 70;
  static const float H = 1;
  static float Q = 5;
  static float P = 0;
  static float U_hat = 0;
  static float K = 0;
  K = P * H / (H * P * H + R);
  U_hat += + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

void setup() {
  Serial.begin (115200);
}

void loop() {
  // Calculate the speed of sound, duration, and distance
  soundsp = (331.4 + (0.606 * temp)) / 10000;
  duration = sonar.ping_median(iterations);
  distance = (duration / 2) * soundsp;

  if (distance <= 400 && distance >= 2) {
    // Valid distance
    kaldist = kalman(distance);
    lpdist = lp.filt(distance);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(", ");
    Serial.print("KalDistance: ");
    Serial.print(kaldist);
    Serial.print(", ");
    Serial.print("LPDistance: ");
    Serial.println(lpdist);
  } else {
    // Invalid distance
    distance = -1;
  }
}

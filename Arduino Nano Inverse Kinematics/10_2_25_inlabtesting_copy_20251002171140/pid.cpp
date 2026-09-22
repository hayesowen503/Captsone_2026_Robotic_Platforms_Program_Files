#include "pid.h"

//// set point variable from kinematics
float actuator_upper_length = 345.0;
float actuator_lower_length = 205.0;
float micro_ros_commanded_length = 275.0;
int used_sp = 0;

//// ---------- Pins ----------
const uint8_t PIN_FB   = A0;  // actuator feedback
const uint8_t PIN_SP   = A1;  // setpoint pot
const uint8_t PIN_IN1  = 10;   // H-bridge IN1 (dir)
const uint8_t PIN_IN2  = 9;   // H-bridge IN2 (dir)
const uint8_t PIN_EN   = 8;   // H-bridge EN (PWM)

//// ---------- PID gains ----------
float Kp = 1.20f;
float Ki = 0.45f;   // per second
float Kd = 0.02f;   // seconds (derivative on measurement)

//// ---------- Control timing ----------
const float    CTRL_HZ = 1000.0f;                        // 500–1000 typical
const uint32_t DT_US   = (uint32_t)(1000000.0f / CTRL_HZ);
uint32_t       next_us = 0;

//// ---------- Limits & shaping ----------
int   ADC_MIN = 60;            // set safely inside your measured endpoints
int   ADC_MAX = 960;
const int PWM_MAX = 255;
const int ERROR_OK_BAND = 2;   // counts (stop when within this)
const float INTEGRAL_MAX = 400.0f;

// Slew: tighter near zero for micro moves
const int SLEW_NEAR = 12;      // pwm counts/tick when |cmd| < 40
const int SLEW_FAR  = 100;     // pwm counts/tick otherwise

// EMA filtering
const float MEAS_EMA_ALPHA = 0.20f;  // 0..1 (higher = more smoothing)
const float SP_EMA_ALPHA   = 0.20f;

// Micro-hold band (brake + tiny I nudge)
const int   MICRO_BAND   = 3;
const float MICRO_I_GAIN = 0.25f;

//// ---------- Stop behavior ----------
const StopMode STOP_MODE = BRAKE;    // better settling for small moves

//// ---------- State ----------
float integral = 0.0f;
int   last_meas = 0;
float dmeas_ema = 0.0f;
float meas_ema  = 0.0f;
float sp_ema    = 0.0f;
int   last_cmd  = 0;

// Smooth start: converts |cmd| (0..255) to PWM with soft offset u0 and gentle slope k
int softDeadzone(int u_abs, int u0, float k){
  if (u_abs <= 0) return 0;
  float x = (float)u_abs / 255.0f;        // 0..1
  float y = k*x + (1.0f - k)*x*x;         // S-ish curve
  int out = (int)(u0 + y*(255 - u0) + 0.5f);
  return out > 255 ? 255 : out;
}

void motorStop(){
  if (STOP_MODE == BRAKE){
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_EN, 0);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_EN, 0);
  }
}

void motorDrive(int cmd){
  cmd = clampi(cmd, -PWM_MAX, PWM_MAX);
  if (cmd > 0){
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    int pwm = softDeadzone(cmd, /*u0=*/14, /*k=*/0.55f);
    analogWrite(PIN_EN, pwm);
  } else if (cmd < 0){
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    int pwm = softDeadzone(-cmd, 14, 0.55f);
    analogWrite(PIN_EN, pwm);
  } else {
    motorStop();
  }
}

void pidSetup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_EN,  OUTPUT);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_EN, 0);


  // Prime filters
  int m0 = analogRead(PIN_FB);
  int s0 = analogRead(PIN_SP);
  meas_ema = (float)m0;
  sp_ema   = (float)s0;
  last_meas = m0;
}

// Scale a float value x from [in_min..in_max] into [out_min..out_max] as an int
int fscale(float x, float in_min, float in_max, int out_min, int out_max) {
  if (in_max == in_min) return (out_min + out_max) / 2; // avoid div/0

  float t = (x - in_min) / (in_max - in_min); // normalize to 0..1
  // Clamp to ensure we don't go out of bounds
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  float out = out_min + t * (out_max - out_min);
  return (int)lroundf(out);  // round to nearest integer
}



void pidLoop() {
  // Fixed-rate scheduler (no threads)
  uint32_t now = micros();
  if ((int32_t)(now - next_us) < 0) return;
  next_us = micros() + DT_US;
  // do { next_us += DT_US; } while ((int32_t)(now - next_us) >= 0);

  // --- Setpoint (filter + clamp to travel band) ---
  // int sp_raw = analogRead(PIN_SP);                      // 0..1023
  // int sp     = map(sp_raw, 0, 1023, ADC_MIN, ADC_MAX);

  // int sp = fscale(micro_ros_commanded_length, actuator_lower_length * 1.2, actuator_upper_length * 0.8, ADC_MIN, ADC_MAX);

  float f_sp_raw = micro_ros_commanded_length * 1000;
  f_sp_raw = clampf(f_sp_raw, actuator_lower_length, actuator_upper_length);
  
  int sp_raw = (int)f_sp_raw;
  int sp     = map(sp_raw, 205, 345, ADC_MIN, ADC_MAX);

  used_sp = sp;

  sp_ema     = SP_EMA_ALPHA * sp + (1.0f - SP_EMA_ALPHA) * sp_ema;
  int setpoint = (int)(sp_ema + 0.5f);
  setpoint = clampi(setpoint, ADC_MIN, ADC_MAX);

  // --- Measurement (filter + soft limits) ---
  int raw  = analogRead(PIN_FB);
  raw      = clampi(raw, ADC_MIN, ADC_MAX);
  meas_ema = MEAS_EMA_ALPHA * raw + (1.0f - MEAS_EMA_ALPHA) * meas_ema;
  int meas = (int)(meas_ema + 0.5f);

  // --- Error & derivative (on measurement) ---
  int   error = setpoint - meas;
  float dmeas = (float)(meas - last_meas) * CTRL_HZ;    // counts/s
  dmeas_ema   = 0.5f * dmeas + 0.5f * dmeas_ema;

  // --- Micro-hold: brake + tiny I to overcome bias ---
  if (abs(error) <= MICRO_BAND){
    motorStop();
    integral += (float)error * (MICRO_I_GAIN / CTRL_HZ);
    integral = clampf(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    last_cmd  = 0;
    last_meas = meas;
    // // Telemetry ~25 Hz
    // static uint16_t acc0=0;
    // if (++acc0 >= (uint16_t)(CTRL_HZ/25)){
    //   acc0 = 0;
    //   Serial.print(F("meas=")); Serial.print(meas);
    //   Serial.print(F(" sp="));   Serial.print(setpoint);
    //   Serial.print(F(" err="));  Serial.print(error);
    //   Serial.print(F(" cmd="));  Serial.print(0);
    //   Serial.print(F(" I="));    Serial.print(integral, 2);
    //   Serial.print(F(" raw_fb=")); Serial.print(raw);
    //   Serial.print(F(" raw_sp=")); Serial.print(sp_raw);
    //   Serial.println();
    // }
    return;
  }

  // --- Integral separation: integrate when moving or away from zero ---
  if (abs(error) > 3 || abs(last_cmd) > 0){
    integral += (float)error / CTRL_HZ;                 // counts*s
    integral = clampf(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  } else {
    integral *= 0.985f;                                 // slight bleed near zero
  }

  // --- Conditional derivative: soften near zero error ---
  float d_term = Kd * dmeas_ema;
  if (abs(error) < 6) d_term *= 0.35f;

  // --- PID control law (counts -> "u") ---
  float u = Kp*(float)error + Ki*integral - d_term;

  // --- Counts -> PWM heuristic (tune for your travel span) ---
  const float COUNTS_TO_PWM = 255.0f / 800.0f;
  int cmd = (int)(u * COUNTS_TO_PWM);

  // --- Slew limit (tighter near zero) ---
  int slew = (abs(last_cmd) < 40) ? SLEW_NEAR : SLEW_FAR;
  int dc = cmd - last_cmd;
  if (dc >  slew) dc =  slew;
  if (dc < -slew) dc = -slew;
  cmd = last_cmd + dc;

  // --- On-target band: stop + integral bleed ---
  if (abs(error) <= ERROR_OK_BAND){
    cmd = 0;
    integral *= 0.95f;
  }

  // --- Anti-windup leak if saturating wrong-way ---
  if ((cmd >= PWM_MAX && error > 0) || (cmd <= -PWM_MAX && error < 0)){
    integral *= 0.99f;
  }

  // --- Drive & book-keeping ---
  motorDrive(cmd);
  last_meas = meas;
  last_cmd  = cmd;

  // // --- Telemetry ~25 Hz ---
  // static uint16_t acc=0;
  // if (++acc >= (uint16_t)(CTRL_HZ/25)){
  //   acc = 0;
  //   Serial.print(F("meas=")); Serial.print(meas);
  //   Serial.print(F(" sp="));   Serial.print(setpoint);
  //   Serial.print(F(" err="));  Serial.print(error);
  //   Serial.print(F(" cmd="));  Serial.print(cmd);
  //   Serial.print(F(" I="));    Serial.print(integral, 2);
  //   Serial.print(F(" raw_fb=")); Serial.print(raw);
  //   Serial.print(F(" raw_sp=")); Serial.print(sp_raw);
  //   Serial.println();
  // }  
}






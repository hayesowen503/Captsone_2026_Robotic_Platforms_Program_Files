


#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

extern float actuator_upper_length;
extern float actuator_lower_length;
extern float micro_ros_commanded_length;
extern int used_sp;

//// ---------- Pins ----------
extern const uint8_t PIN_FB;  // actuator feedback
extern const uint8_t PIN_SP;  // setpoint pot
extern const uint8_t PIN_IN1;   // H-bridge IN1 (dir)
extern const uint8_t PIN_IN2;   // H-bridge IN2 (dir)
extern const uint8_t PIN_EN;   // H-bridge EN (PWM)

//// ---------- PID gains ----------
extern float Kp;
extern float Ki;   // per second
extern float Kd;   // seconds (derivative on measurement)

//// ---------- Control timing ----------
extern const float    CTRL_HZ;                        // 500–1000 typical
extern const uint32_t DT_US;
extern uint32_t       next_us;

//// ---------- Limits & shaping ----------
extern int   ADC_MIN;            // set safely inside your measured endpoints
extern int   ADC_MAX;
extern const int PWM_MAX;
extern const int ERROR_OK_BAND;   // counts (stop when within this)
extern const float INTEGRAL_MAX;

// Slew: tighter near zero for micro moves
extern const int SLEW_NEAR;      // pwm counts/tick when |cmd| < 40
extern const int SLEW_FAR;     // pwm counts/tick otherwise

// EMA filtering
extern const float MEAS_EMA_ALPHA;  // 0..1 (higher = more smoothing)
extern const float SP_EMA_ALPHA;

// Micro-hold band (brake + tiny I nudge)
extern const int   MICRO_BAND;
extern const float MICRO_I_GAIN;

//// ---------- Stop behavior ----------
enum StopMode { COAST, BRAKE };
extern const StopMode STOP_MODE;    // better settling for small moves

//// ---------- State ----------
extern float integral;
extern int   last_meas;
extern float dmeas_ema;
extern float meas_ema;
extern float sp_ema;
extern int   last_cmd;

//// ---------- Helpers ----------
static inline int   clampi(int x, int lo, int hi){ return x<lo?lo:(x>hi?hi:x); }
static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }

int softDeadzone(int u_abs, int u0, float k);

void motorStop();

void motorDrive(int cmd);

void pidSetup();

void pidLoop();

int fscale(float x, float in_min, float in_max, int out_min, int out_max);
#endif
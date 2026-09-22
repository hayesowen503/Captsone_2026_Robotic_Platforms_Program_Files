#include "kinematics.h"

// ----------- Geometry (EDIT for your rig) -----------
// Units: millimeters. Right-handed base frame.
// Base attachment points Bi in base frame B (mm)
 const float Bx[3] = { -80.0f,  80.0f,   0.0f };
 const float By[3] = { -46.2f, -46.2f,  92.4f };
 const float Bz[3] = {   0.0f,   0.0f,   0.0f };

// Platform attachment points Pi in platform frame P (mm)
 const float Px[3] = { -65.0f,  65.0f,   0.0f };
 const float Py[3] = { -37.6f, -37.6f,  75.2f };
 const float Pz[3] = {   0.0f,   0.0f,   0.0f };

// Home pose (mm, deg)
 const float Z_HOME_MM = 120.0f;

// Safety limits
 const float Z_MIN_MM =  90.0f;
 const float Z_MAX_MM = 150.0f;
 const float ROLL_MAX_DEG  = 15.0f;
 const float PITCH_MAX_DEG = 15.0f;

// Utility
inline float deg2rad(float d) { return d * 3.14159265359f / 180.0f; }
float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Compute rotation matrix R = Ry(pitch) * Rx(roll) for 3‑DOF tilt
void rpy_to_R(float roll_deg, float pitch_deg, float R[9]) {
  const float cr = cosf(deg2rad(roll_deg));
  const float sr = sinf(deg2rad(roll_deg));
  const float cp = cosf(deg2rad(pitch_deg));
  const float sp = sinf(deg2rad(pitch_deg));
  // R = Ry * Rx
  R[0] = cp;   R[1] = sp * sr;  R[2] = sp * cr;
  R[3] = 0.0f; R[4] = cr;       R[5] = -sr;
  R[6] = -sp;  R[7] = cp * sr;  R[8] = cp * cr;
}

// Transform P point to base frame given pose {z, roll, pitch}
void transform_point(const float R[9], float z_mm, float px, float py, float pz, float &x, float &y, float &z) {
  // Platform origin relative to base
  const float Tx = 0.0f;
  const float Ty = 0.0f;
  const float Tz = z_mm;
  // R * p + T
  x = R[0]*px + R[1]*py + R[2]*pz + Tx;
  y = R[3]*px + R[4]*py + R[5]*pz + Ty;
  z = R[6]*px + R[7]*py + R[8]*pz + Tz;
}

// Inverse kinematics: returns actuator lengths (straight-line Bi->Pi')
void ik_lengths(float z_mm, float roll_deg, float pitch_deg, float L[3]) {
  float R[9];
  rpy_to_R(roll_deg, pitch_deg, R);
  for (int i = 0; i < 3; ++i) {
    float px_b, py_b, pz_b; // platform point in base frame
    transform_point(R, z_mm, Px[i], Py[i], Pz[i], px_b, py_b, pz_b);
    const float dx = px_b - Bx[i];
    const float dy = py_b - By[i];
    const float dz = pz_b - Bz[i];
    L[i] = sqrtf(dx*dx + dy*dy + dz*dz);
  }
}
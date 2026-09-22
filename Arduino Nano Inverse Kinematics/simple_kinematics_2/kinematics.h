
#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <math.h>

// ----------- Geometry (EDIT for your rig) -----------
// Units: millimeters. Right-handed base frame.
// Base attachment points Bi in base frame B (mm)
extern  const float Bx[3];
extern  const float By[3];
extern  const float Bz[3];

// Platform attachment points Pi in platform frame P (mm)
extern  const float Px[3];
extern  const float Py[3];
extern  const float Pz[3];

// Home pose (mm, deg)
extern  const float Z_HOME_MM;

// Safety limits
extern  const float Z_MIN_MM;
extern  const float Z_MAX_MM;
extern  const float ROLL_MAX_DEG;
extern  const float PITCH_MAX_DEG;

// Utility
inline float deg2rad(float d);
float clampf(float x, float lo, float hi);

// Compute rotation matrix R = Ry(pitch) * Rx(roll) for 3‑DOF tilt
void rpy_to_R(float roll_deg, float pitch_deg, float R[9]);

// Transform P point to base frame given pose {z, roll, pitch}
void transform_point(const float R[9], float z_mm, float px, float py, float pz, float &x, float &y, float &z);

// Inverse kinematics: returns actuator lengths (straight-line Bi->Pi')
void ik_lengths(float z_mm, float roll_deg, float pitch_deg, float L[3]);

#endif
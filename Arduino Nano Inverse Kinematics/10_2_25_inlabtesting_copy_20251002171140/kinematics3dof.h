#ifndef KINEMATICS3DOF_H
#define KINEMATICS3DOF_H

#include <Arduino.h>   // for PI in Arduino builds
#include <stddef.h>

#ifndef PI
// Fallback if not compiling under Arduino core
#define PI 3.14159265358979323846f
#endif

// -------- Physical parameters (EDIT THESE) --------
// Units: meters and radians
extern float KIN_RB;      // base radius (m)
extern float KIN_RP;      // platform radius (m)
extern float KIN_H0;      // home height (m)
extern float KIN_ALPHA;   // platform triangle rotation vs base (rad)

// Enable simple stroke limits by setting to 1
#define KIN_USE_LIMITS 0
extern const float KIN_LMIN[3];
extern const float KIN_LMAX[3];

// Compute actuator lengths for pose
//   dz    : height offset from home (m)
//   theta : tilt magnitude (rad)
//   psi   : tilt direction azimuth from +X (rad)
//   L_out[3] : actuator lengths (m) for i=0..2
// returns: 1 = ok, 0 = violated limits (if enabled)
int kin3_lengths_simple(float dz, float theta, float psi, float L_out[3]);

#endif // KINEMATICS3DOF_H

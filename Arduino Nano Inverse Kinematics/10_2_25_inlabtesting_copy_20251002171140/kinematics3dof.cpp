#include "kinematics3dof.h"
#include <math.h>

// ====== Global parameters (owning definitions) ======
float KIN_RB    = 0.100f;
float KIN_RP    = 0.100f;
float KIN_H0    = 0.275f;
float KIN_ALPHA = PI / 6.0f;
// float KIN_ALPH = 0;

const float KIN_LMIN[3] = {0.100f, 0.100f, 0.100f};
const float KIN_LMAX[3] = {0.160f, 0.160f, 0.160f};

// ===== Helpers =====
static void mat3_mul(const float A[9], const float B[9], float C[9]) {
  for (int i=0;i<3;i++) {
    for (int j=0;j<3;j++) {
      C[3*i+j] = A[3*i+0]*B[0*3+j] + A[3*i+1]*B[1*3+j] + A[3*i+2]*B[2*3+j];
    }
  }
}

static void mat3_mul_vec3(const float M[9], const float v[3], float r[3]) {
  r[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
  r[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
  r[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
}

static void fill_triangle(float r, float a, float M_out[9]) {
  for (int k=0;k<3;k++) {
    float ang = a + (2.0f*PI/3.0f) * k;
    float c = cosf(ang), s = sinf(ang);
    M_out[0*3 + k] = r * c;
    M_out[1*3 + k] = r * s;
    M_out[2*3 + k] = 0.0f;
  }
}

// ===== Main function =====
int kin3_lengths_simple(float dz, float theta, float psi, float L_out[3]) {
  if (!L_out) return 0;

  // Anchors
  float B[9], P[9];
  fill_triangle(KIN_RB,    0.0f,    B);
  fill_triangle(KIN_RP, KIN_ALPHA,  P);

  // Rotation R = Rz(psi) * Ry(theta) * Rz(-psi)
  float cps = cosf(psi),  sps = sinf(psi);
  float ct  = cosf(theta), st  = sinf(theta);

  float Rz_p[9] = { cps,-sps,0,  sps,cps,0,  0,0,1 };
  float Ry[9]   = {  ct,0,st,   0,1,0,     -st,0,ct };
  float Rz_m[9] = { cps, sps,0, -sps,cps,0, 0,0,1 };

  float Rt[9], R[9];
  mat3_mul(Rz_p, Ry, Rt);
  mat3_mul(Rt, Rz_m, R);

  // Translation
  float Tz = KIN_H0 + dz;

  // Lengths
  for (int i=0;i<3;i++) {
    float Pi[3] = { P[0*3+i], P[1*3+i], P[2*3+i] };
    float Pw[3];
    mat3_mul_vec3(R, Pi, Pw);
    Pw[2] += Tz;

    float vx = Pw[0] - B[0*3+i];
    float vy = Pw[1] - B[1*3+i];
    float vz = Pw[2] - B[2*3+i];
    float Li = sqrtf(vx*vx + vy*vy + vz*vz);

#if KIN_USE_LIMITS
    if (Li < KIN_LMIN[i] || Li > KIN_LMAX[i]) {
      L_out[i] = Li;
      return 0;
    }
#endif
    L_out[i] = Li;
  }
  return 1;
}

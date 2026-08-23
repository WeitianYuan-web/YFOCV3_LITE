#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_PI                 3.14159265358979323846f
#define FOC_TWO_PI             6.28318530717958647692f
#define FOC_SQRT3_2            0.86602540378443864676f
#define FOC_PWM_NEUTRAL_DUTY   0.5f

typedef struct
{
  float alpha;
  float beta;
} Foc_AlphaBeta_t;

typedef struct
{
  float d;
  float q;
} Foc_Dq_t;

void Foc_MathInit(void);
float Foc_Clamp(float in, float min_v, float max_v);
float Foc_WrapAngle0To2Pi(float angle_rad);
float Foc_WrapAngleToPi(float angle_rad);
float Foc_ApplyEncoderDirToMechTheta(float mech_theta_raw, int8_t encoder_dir);
float Foc_ApplyEncoderDirToMechVelocity(float vel_mech_rad_s, int8_t encoder_dir);
Foc_AlphaBeta_t Foc_InversePark(Foc_Dq_t dq, float electrical_angle_rad);
void Foc_LimitDQ(float max_v, float *d_v, float *q_v);
void Foc_SinCos(float angle_rad, float *sin_out, float *cos_out);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */

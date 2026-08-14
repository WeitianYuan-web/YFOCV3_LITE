#include "foc_math.h"

#include <math.h>

static void Foc_SinCosFpu(float angle_rad, float *sin_out, float *cos_out)
{
  const float theta = Foc_WrapAngleToPi(angle_rad);
  *sin_out = sinf(theta);
  *cos_out = cosf(theta);
}

float Foc_Clamp(float in, float min_v, float max_v)
{
  if (in < min_v)
  {
    return min_v;
  }
  if (in > max_v)
  {
    return max_v;
  }
  return in;
}

float Foc_WrapAngle0To2Pi(float angle_rad)
{
  if ((angle_rad >= 0.0f) && (angle_rad < FOC_TWO_PI))
  {
    return angle_rad;
  }

  angle_rad -= FOC_TWO_PI * floorf(angle_rad / FOC_TWO_PI);

  if (angle_rad >= FOC_TWO_PI)
  {
    angle_rad -= FOC_TWO_PI;
  }
  else if (angle_rad < 0.0f)
  {
    angle_rad += FOC_TWO_PI;
  }

  return angle_rad;
}

float Foc_WrapAngleToPi(float angle_rad)
{
  if ((angle_rad >= -FOC_PI) && (angle_rad < FOC_PI))
  {
    return angle_rad;
  }

  angle_rad -= FOC_TWO_PI * floorf((angle_rad + FOC_PI) / FOC_TWO_PI);

  if (angle_rad >= FOC_PI)
  {
    angle_rad -= FOC_TWO_PI;
  }
  else if (angle_rad < -FOC_PI)
  {
    angle_rad += FOC_TWO_PI;
  }

  return angle_rad;
}

float Foc_ApplyEncoderDirToMechTheta(float mech_theta_raw, int8_t encoder_dir)
{
  return Foc_WrapAngle0To2Pi(mech_theta_raw * (float)encoder_dir);
}

float Foc_ApplyEncoderDirToMechVelocity(float vel_mech_rad_s, int8_t encoder_dir)
{
  return vel_mech_rad_s * (float)encoder_dir;
}

Foc_AlphaBeta_t Foc_InversePark(Foc_Dq_t dq, float electrical_angle_rad)
{
  float sin_theta;
  float cos_theta;
  Foc_AlphaBeta_t alpha_beta;

  Foc_SinCos(electrical_angle_rad, &sin_theta, &cos_theta);

  alpha_beta.alpha = dq.d * cos_theta - dq.q * sin_theta;
  alpha_beta.beta = dq.d * sin_theta + dq.q * cos_theta;

  return alpha_beta;
}

void Foc_LimitDQ(float max_v, float *d_v, float *q_v)
{
  const float max_v_sq = max_v * max_v;
  const float norm_sq = (*d_v) * (*d_v) + (*q_v) * (*q_v);

  if (norm_sq <= max_v_sq)
  {
    return;
  }

  *d_v = Foc_Clamp(*d_v, -max_v, max_v);
  {
    const float remain = max_v_sq - (*d_v) * (*d_v);
    const float q_max = (remain > 0.0f) ? sqrtf(remain) : 0.0f;
    *q_v = Foc_Clamp(*q_v, -q_max, q_max);
  }
}

void Foc_SinCos(float angle_rad, float *sin_out, float *cos_out)
{
  if ((sin_out == 0) || (cos_out == 0))
  {
    return;
  }
  Foc_SinCosFpu(angle_rad, sin_out, cos_out);
}

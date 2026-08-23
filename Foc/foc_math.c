#include "foc_math.h"

#include <math.h>

#define FOC_COS_LUT_N        (256U)
#define FOC_INV_TWO_PI       (0.15915494309189535f)

static float s_cos_lut[FOC_COS_LUT_N + 1U];

void Foc_MathInit(void)
{
  uint32_t i;
  const float step = FOC_TWO_PI / (float)FOC_COS_LUT_N;

  for (i = 0U; i <= FOC_COS_LUT_N; i++)
  {
    s_cos_lut[i] = cosf((float)i * step);
  }
}

static float Foc_WrapUnit(float turns)
{
  const int32_t n = (int32_t)turns;
  float f = turns - (float)n;

  if (f < 0.0f)
  {
    f += 1.0f;
  }
  else if (f >= 1.0f)
  {
    f -= 1.0f;
  }
  return f;
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
  return Foc_WrapUnit(angle_rad * FOC_INV_TWO_PI) * FOC_TWO_PI;
}

float Foc_WrapAngleToPi(float angle_rad)
{
  if ((angle_rad >= -FOC_PI) && (angle_rad < FOC_PI))
  {
    return angle_rad;
  }
  {
    float a = Foc_WrapAngle0To2Pi(angle_rad);
    if (a >= FOC_PI)
    {
      a -= FOC_TWO_PI;
    }
    return a;
  }
}

float Foc_ApplyEncoderDirToMechTheta(float mech_theta_raw, int8_t encoder_dir)
{
  if (encoder_dir >= 0)
  {
    return mech_theta_raw;
  }
  if (mech_theta_raw <= 0.0f)
  {
    return 0.0f;
  }
  return FOC_TWO_PI - mech_theta_raw;
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
  const float scale = (float)FOC_COS_LUT_N * FOC_INV_TWO_PI;
  float x;
  int32_t i;
  float f;
  uint32_t idx;
  uint32_t idxs;

  if ((sin_out == 0) || (cos_out == 0))
  {
    return;
  }

  x = angle_rad * scale;
  i = (int32_t)x;
  f = x - (float)i;
  if (f < 0.0f)
  {
    f += 1.0f;
    i -= 1;
  }
  idx = (uint32_t)i & (FOC_COS_LUT_N - 1U);
  idxs = (idx + (FOC_COS_LUT_N - (FOC_COS_LUT_N / 4U))) & (FOC_COS_LUT_N - 1U);
  *cos_out = s_cos_lut[idx] + (f * (s_cos_lut[idx + 1U] - s_cos_lut[idx]));
  *sin_out = s_cos_lut[idxs] + (f * (s_cos_lut[idxs + 1U] - s_cos_lut[idxs]));
}

#include "foc_svpwm.h"
#include "foc_math.h"

static float Foc_SvmClamp01f(float in)
{
  return Foc_Clamp(in, 0.0f, 1.0f);
}

void Foc_SvpwmOffsetOptimized(float u_alpha,
                              float u_beta,
                              float *duty_a,
                              float *duty_b,
                              float *duty_c)
{
  float ua;
  float ub;
  float uc;
  float u_max;
  float u_min;
  float u_offset;

  ua = u_alpha;
  ub = -0.5f * u_alpha + FOC_SQRT3_2 * u_beta;
  uc = -0.5f * u_alpha - FOC_SQRT3_2 * u_beta;

  u_max = ua > ub ? (ua > uc ? ua : uc) : (ub > uc ? ub : uc);
  u_min = ua < ub ? (ua < uc ? ua : uc) : (ub < uc ? ub : uc);

  u_offset = -(u_max + u_min) * 0.5f;

  *duty_a = Foc_SvmClamp01f(ua + u_offset + FOC_PWM_NEUTRAL_DUTY);
  *duty_b = Foc_SvmClamp01f(ub + u_offset + FOC_PWM_NEUTRAL_DUTY);
  *duty_c = Foc_SvmClamp01f(uc + u_offset + FOC_PWM_NEUTRAL_DUTY);
}

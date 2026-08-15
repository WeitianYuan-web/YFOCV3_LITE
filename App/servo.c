#include "servo.h"

#include "config.h"
#include "encoder.h"
#include "foc_math.h"
#include "foc_svpwm.h"
#include "pwm.h"
#include "stm32g4xx.h"

static Foc_Encoder_t s_enc;
static volatile ServoMode_t s_mode = SERVO_IDLE;

static volatile float s_p_set;
static volatile float s_v_set;
static volatile float s_kp;
static volatile float s_kd;
static volatile float s_t_ff;
static volatile float s_t_ref;
static volatile int8_t s_closed_loop_dir = 1;

static volatile float s_ol_d;
static volatile float s_ol_q;
static volatile float s_ol_rate;
static volatile float s_ol_elec;

static volatile float s_volt_d;
static volatile float s_volt_q;
static float s_d_out;
static float s_q_out;

static float Servo_Slew(float current, float target, float max_step)
{
  const float delta = target - current;

  if (delta > max_step)
  {
    return current + max_step;
  }
  if (delta < -max_step)
  {
    return current - max_step;
  }
  return target;
}

static void Servo_ApplyDq(float d_v, float q_v, float elec)
{
  Foc_Dq_t dq;
  Foc_AlphaBeta_t ab;
  float duty_a;
  float duty_b;
  float duty_c;
  const float max_step = CFG_V_SLEW_PU_S * CFG_PWM_DT_S;

  dq.d = d_v;
  dq.q = q_v;
  Foc_LimitDQ(CFG_V_LIMIT, &dq.d, &dq.q);
  s_d_out = Servo_Slew(s_d_out, dq.d, max_step);
  s_q_out = Servo_Slew(s_q_out, dq.q, max_step);
  dq.d = s_d_out;
  dq.q = s_q_out;
  ab = Foc_InversePark(dq, elec);
  Foc_SvpwmOffsetOptimized(ab.alpha, ab.beta, &duty_a, &duty_b, &duty_c);
  Pwm_ApplyDuty(duty_a, duty_b, duty_c);
}

void Servo_Init(void)
{
  const float wn = 2.0f * FOC_PI * CFG_VEL_PLL_HZ;
  Foc_EncoderConfig_t cfg;

  cfg.pole_pairs = (uint8_t)CFG_POLE_PAIRS;
  cfg.direction = 1;
  cfg.electrical_offset_rad = 0.0f;
  cfg.pll_kp = 2.0f * CFG_VEL_PLL_ZETA * wn;
  cfg.pll_ki = wn * wn;

  Foc_EncoderInit(&s_enc, &cfg);
  s_mode = SERVO_IDLE;
  s_closed_loop_dir = 1;
  s_p_set = 0.0f;
  s_v_set = 0.0f;
  s_kp = 0.0f;
  s_kd = 0.0f;
  s_t_ff = 0.0f;
  s_t_ref = 0.0f;
  s_d_out = 0.0f;
  s_q_out = 0.0f;
}

void Servo_OnPwmIsr(void)
{
  uint16_t raw;
  ServoMode_t mode = s_mode;

  if (Encoder_ReadRawFast(&raw) != 0U)
  {
    const float mech = ((float)raw * FOC_TWO_PI) / (float)ENCODER_CPR;
    (void)Foc_EncoderUpdate(&s_enc, mech, CFG_PWM_DT_S);
  }
  else
  {
    Foc_EncoderPredict(&s_enc, CFG_PWM_DT_S);
  }

  if (mode == SERVO_OPENLOOP)
  {
    s_ol_elec = Foc_WrapAngle0To2Pi(s_ol_elec + (s_ol_rate * CFG_PWM_DT_S));
    Servo_ApplyDq(s_ol_d, s_ol_q, s_ol_elec);
  }
  else if (mode == SERVO_VOLTAGE)
  {
    Servo_ApplyDq(s_volt_d, s_volt_q, Foc_EncoderGetElectrical(&s_enc));
  }
  else if (mode == SERVO_RUN)
  {
    const float q = (float)s_closed_loop_dir * s_t_ref;
    Servo_ApplyDq(0.0f, q, Foc_EncoderGetElectrical(&s_enc));
  }
}

void Servo_OnCtrlIsr(void)
{
  float p_act;
  float v_act;
  float t_ref;

  if (s_mode != SERVO_RUN)
  {
    return;
  }

  p_act = Foc_EncoderGetPosition(&s_enc);
  v_act = Foc_EncoderGetVelocity(&s_enc);
  t_ref = (s_kd * (s_v_set - v_act)) + (s_kp * (s_p_set - p_act)) + s_t_ff;
  s_t_ref = Foc_Clamp(t_ref, -CFG_V_LIMIT, CFG_V_LIMIT);
}

void Servo_SetMode(ServoMode_t mode)
{
  s_mode = mode;
}

ServoMode_t Servo_GetMode(void)
{
  return s_mode;
}

void Servo_SetOpenloop(float d_v, float q_v, float elec_rate_rad_s, float elec_angle_rad)
{
  s_ol_d = d_v;
  s_ol_q = q_v;
  s_ol_rate = elec_rate_rad_s;
  s_ol_elec = Foc_WrapAngle0To2Pi(elec_angle_rad);
  s_mode = SERVO_OPENLOOP;
}

void Servo_SetVoltageCmd(float d_v, float q_v)
{
  s_volt_d = d_v;
  s_volt_q = q_v;
  s_mode = SERVO_VOLTAGE;
}

void Servo_SetMotion(float p_set, float v_set, float t_ff)
{
  s_p_set = p_set;
  s_v_set = Foc_Clamp(v_set, CFG_VEL_CMD_MIN, CFG_VEL_CMD_MAX);
  s_t_ff = t_ff;
}

void Servo_SetGains(float kp, float kd)
{
  s_kp = Foc_Clamp(kp, CFG_KP_MIN, CFG_KP_MAX);
  s_kd = Foc_Clamp(kd, CFG_KD_MIN, CFG_KD_MAX);
}

void Servo_SetZero(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  Foc_EncoderSetZero(&s_enc);
  s_p_set = 0.0f;
  __set_PRIMASK(primask);
}

void Servo_SetClosedLoopDir(int8_t dir)
{
  s_closed_loop_dir = (dir < 0) ? -1 : 1;
}

void Servo_SetEncoderAlignment(int8_t encoder_dir, float electrical_offset_rad)
{
  Foc_EncoderSetAlignment(&s_enc, encoder_dir, electrical_offset_rad);
}

void Servo_SetPolePairs(uint8_t pole_pairs)
{
  Foc_EncoderSetPolePairs(&s_enc, pole_pairs);
}

void Servo_HoldPosition(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_p_set = Foc_EncoderGetPosition(&s_enc);
  s_v_set = 0.0f;
  s_t_ff = 0.0f;
  s_t_ref = 0.0f;
  __set_PRIMASK(primask);
}

void Servo_GetTelemetry(ServoTelemetry_t *out)
{
  if (out == 0)
  {
    return;
  }
  out->p_set = s_p_set;
  out->v_set = s_v_set;
  out->kp = s_kp;
  out->kd = s_kd;
  out->t_ff = s_t_ff;
  out->t_ref = s_t_ref;
  out->p_act = Foc_EncoderGetPosition(&s_enc);
  out->v_act = Foc_EncoderGetVelocity(&s_enc);
  out->elec = Foc_EncoderGetElectrical(&s_enc);
  out->closed_loop_dir = s_closed_loop_dir;
}

Foc_Encoder_t *Servo_GetEncoder(void)
{
  return &s_enc;
}

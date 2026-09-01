#include "servo.h"

#include "config.h"
#include "encoder.h"
#include "foc_math.h"
#include "foc_svpwm.h"
#include "pwm.h"
#include "board.h"

#include <math.h>

static Foc_Encoder_t s_enc;
static volatile ServoMode_t s_mode = SERVO_IDLE;
static volatile uint8_t s_ctrl_mode = SERVO_CTRL_MOTION;
static volatile uint8_t s_cmd_valid = 0U;

static volatile float s_p_set;
static volatile float s_v_set;
static volatile float s_v_max;
static volatile float s_kp;
static volatile float s_kd;
static volatile float s_t_ff;
static volatile float s_t_ref;
static volatile float s_vel_kp;
static volatile float s_vel_ki;
static volatile float s_pos_kp;
static volatile float s_pos_ki;
static volatile float s_pos_kd;
static volatile int8_t s_closed_loop_dir = 1;

static float s_vel_i;
static float s_pos_i;

static volatile float s_ol_d;
static volatile float s_ol_q;
static volatile float s_ol_rate;
static volatile float s_ol_elec;

static volatile float s_volt_d;
static volatile float s_volt_q;
static float s_d_out;
static float s_q_out;

static void Servo_ClearLoopState(void)
{
  s_vel_i = 0.0f;
  s_pos_i = 0.0f;
  s_t_ref = 0.0f;
  s_cmd_valid = 0U;
}

static float Servo_PiLimited(float *i, float err, float kp, float ki, float dt, float lim)
{
  *i += ki * err * dt;
  *i = Foc_Clamp(*i, -lim, lim);
  return Foc_Clamp((kp * err) + *i, -lim, lim);
}

static float Servo_PosSpeedLimit(float ep, float vmax, float acc)
{
  float lim = vmax;

  if ((acc > 0.0f) && (lim > 0.0f))
  {
    const float v_dec = sqrtf(2.0f * acc * fabsf(ep));
    if (v_dec < lim)
    {
      lim = v_dec;
    }
  }
  return lim;
}

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
  Foc_EncoderConfig_t cfg;

  Foc_MathInit();
  cfg.pole_pairs = (uint8_t)CFG_POLE_PAIRS;
  cfg.direction = 1;
  cfg.electrical_offset_rad = 0.0f;
  cfg.vel_lpf_hz = CFG_VEL_LPF_HZ;
  cfg.sample_hz = (float)CFG_PWM_HZ;

  Foc_EncoderInit(&s_enc, &cfg);
  s_mode = SERVO_IDLE;
  s_ctrl_mode = SERVO_CTRL_MOTION;
  s_closed_loop_dir = 1;
  s_p_set = 0.0f;
  s_v_set = 0.0f;
  s_v_max = CFG_POS_VMAX_DEFAULT;
  s_kp = 0.0f;
  s_kd = 0.0f;
  s_t_ff = 0.0f;
  s_t_ref = 0.0f;
  s_vel_kp = Foc_Clamp(CFG_VEL_KP_DEFAULT, 0.0f, CFG_KP_VEL_MAX);
  s_vel_ki = Foc_Clamp(CFG_VEL_KI_DEFAULT, 0.0f, CFG_KI_VEL_MAX);
  s_pos_kp = Foc_Clamp(CFG_POS_KP_DEFAULT, 0.0f, CFG_KP_POS_MAX);
  s_pos_ki = Foc_Clamp(CFG_POS_KI_DEFAULT, 0.0f, CFG_KI_POS_MAX);
  s_pos_kd = Foc_Clamp(CFG_POS_KD_DEFAULT, 0.0f, CFG_KD_POS_MAX);
  s_vel_i = 0.0f;
  s_pos_i = 0.0f;
  s_cmd_valid = 0U;
  s_d_out = 0.0f;
  s_q_out = 0.0f;
}

static void Servo_ApplyEncoderSample(uint8_t ok, uint16_t raw)
{
  if (ok != 0U)
  {
    const float mech = ((float)raw * FOC_TWO_PI) / (float)ENCODER_CPR;
    (void)Foc_EncoderUpdate(&s_enc, mech, CFG_PWM_DT_S);
  }
  else
  {
    Foc_EncoderPredict(&s_enc, CFG_PWM_DT_S);
  }
}

void Servo_OnPwmIsr(void)
{
  uint16_t raw;
  uint8_t ok;
  ServoMode_t mode = s_mode;

  ok = Encoder_ReadRawFast(&raw);
  Servo_ApplyEncoderSample(ok, raw);

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
  const float dt = CFG_CTRL_DT_S;
  const float vlim = CFG_V_LIMIT;

  if (s_mode != SERVO_RUN)
  {
    return;
  }
  if (s_cmd_valid == 0U)
  {
    s_t_ref = 0.0f;
    return;
  }

  p_act = Foc_EncoderGetPosition(&s_enc);
  v_act = Foc_EncoderGetVelocity(&s_enc);

  if (s_ctrl_mode == SERVO_CTRL_MOTION)
  {
    const float t_ref = (s_kd * (s_v_set - v_act)) + (s_kp * (s_p_set - p_act)) + s_t_ff;
    s_t_ref = Foc_Clamp(t_ref, -vlim, vlim);
  }
  else if (s_ctrl_mode == SERVO_CTRL_VELOCITY)
  {
    s_t_ref = Servo_PiLimited(&s_vel_i, s_v_set - v_act, s_vel_kp, s_vel_ki, dt, vlim);
  }
  else
  {
    const float ep = s_p_set - p_act;
    const float d_term = -(s_pos_kd * v_act);
    const float vmax = s_v_max;
    const float v_lim = Servo_PosSpeedLimit(ep, vmax, CFG_POS_ACC_DEFAULT);
    const float v_prof = (ep >= 0.0f) ? v_lim : -v_lim;
    const float settle = CFG_POS_SETTLE_RAD;
    float v_pid;
    float v_ref;
    float inner_unsat;
    float pos_i_old;
    float w;

    pos_i_old = s_pos_i;
    if (vmax <= 0.0f)
    {
      s_pos_i = 0.0f;
      v_ref = 0.0f;
    }
    else
    {
      if (s_pos_ki <= 0.0f)
      {
        s_pos_i = 0.0f;
        v_pid = (s_pos_kp * ep) + d_term;
      }
      else
      {
        s_pos_i += s_pos_ki * ep * dt;
        s_pos_i = Foc_Clamp(s_pos_i, -vmax, vmax);
        v_pid = (s_pos_kp * ep) + s_pos_i + d_term;
      }
      v_pid = Foc_Clamp(v_pid, -vmax, vmax);
      if (fabsf(ep) >= settle)
      {
        w = 1.0f;
      }
      else if (settle <= 0.0f)
      {
        w = 0.0f;
      }
      else
      {
        w = fabsf(ep) / settle;
      }
      /* Far: follow vmax / sqrt(2 a |ep|). Near: hand off to PID. */
      v_ref = (w * v_prof) + ((1.0f - w) * v_pid);
      v_ref = Foc_Clamp(v_ref, -vmax, vmax);
    }

    inner_unsat = (s_vel_kp * (v_ref - v_act)) + s_vel_i + (s_vel_ki * (v_ref - v_act) * dt);
    s_t_ref = Servo_PiLimited(&s_vel_i, v_ref - v_act, s_vel_kp, s_vel_ki, dt, vlim);
    if ((inner_unsat > vlim) || (inner_unsat < -vlim))
    {
      s_pos_i = pos_i_old;
    }
  }
}

void Servo_SetMode(ServoMode_t mode)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (mode != SERVO_RUN)
  {
    Servo_ClearLoopState();
  }
  s_mode = mode;
  __set_PRIMASK(primask);
}

ServoMode_t Servo_GetMode(void)
{
  return s_mode;
}

void Servo_SetOpenloop(float d_v, float q_v, float elec_rate_rad_s, float elec_angle_rad)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_ol_d = d_v;
  s_ol_q = q_v;
  s_ol_rate = elec_rate_rad_s;
  s_ol_elec = Foc_WrapAngle0To2Pi(elec_angle_rad);
  /* Cali/openloop must not slew from the previous closed-loop voltage. */
  s_d_out = d_v;
  s_q_out = q_v;
  s_mode = SERVO_OPENLOOP;
  __set_PRIMASK(primask);
}

void Servo_SetOpenloopRate(float elec_rate_rad_s)
{
  s_ol_rate = elec_rate_rad_s;
  s_mode = SERVO_OPENLOOP;
}

float Servo_GetOpenloopElec(void)
{
  return s_ol_elec;
}

void Servo_SetVoltageCmd(float d_v, float q_v)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_volt_d = d_v;
  s_volt_q = q_v;
  s_d_out = d_v;
  s_q_out = q_v;
  s_mode = SERVO_VOLTAGE;
  __set_PRIMASK(primask);
}

void Servo_SetMotion(float p_set, float v_set, float t_ff)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_p_set = p_set;
  s_v_set = Foc_Clamp(v_set, CFG_VEL_CMD_MIN, CFG_VEL_CMD_MAX);
  s_t_ff = Foc_Clamp(t_ff, -1.0f, 1.0f);
  s_cmd_valid = 1U;
  __set_PRIMASK(primask);
}

void Servo_SetVelocityCmd(float v_set)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_v_set = Foc_Clamp(v_set, CFG_VEL_CMD_MIN, CFG_VEL_CMD_MAX);
  s_cmd_valid = 1U;
  __set_PRIMASK(primask);
}

void Servo_SetPositionCmd(float p_set, float v_max)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_p_set = p_set;
  s_v_max = Foc_Clamp(v_max, 0.0f, CFG_VEL_CMD_MAX);
  s_cmd_valid = 1U;
  __set_PRIMASK(primask);
}

void Servo_SetGains(float kp, float kd)
{
  s_kp = Foc_Clamp(kp, CFG_KP_MIN, CFG_KP_MAX);
  s_kd = Foc_Clamp(kd, CFG_KD_MIN, CFG_KD_MAX);
}

void Servo_GetGains(float *kp, float *kd)
{
  if (kp != 0)
  {
    *kp = s_kp;
  }
  if (kd != 0)
  {
    *kd = s_kd;
  }
}

void Servo_SetVelocityGains(float kp, float ki)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_vel_kp = Foc_Clamp(kp, 0.0f, CFG_KP_VEL_MAX);
  s_vel_ki = Foc_Clamp(ki, 0.0f, CFG_KI_VEL_MAX);
  if (s_vel_ki == 0.0f)
  {
    s_vel_i = 0.0f;
  }
  __set_PRIMASK(primask);
}

void Servo_GetVelocityGains(float *kp, float *ki)
{
  if (kp != 0)
  {
    *kp = s_vel_kp;
  }
  if (ki != 0)
  {
    *ki = s_vel_ki;
  }
}

void Servo_SetPositionGains(float kp, float ki, float kd)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_pos_kp = Foc_Clamp(kp, 0.0f, CFG_KP_POS_MAX);
  s_pos_ki = Foc_Clamp(ki, 0.0f, CFG_KI_POS_MAX);
  s_pos_kd = Foc_Clamp(kd, 0.0f, CFG_KD_POS_MAX);
  if (s_pos_ki == 0.0f)
  {
    s_pos_i = 0.0f;
  }
  __set_PRIMASK(primask);
}

void Servo_GetPositionGains(float *kp, float *ki, float *kd)
{
  if (kp != 0)
  {
    *kp = s_pos_kp;
  }
  if (ki != 0)
  {
    *ki = s_pos_ki;
  }
  if (kd != 0)
  {
    *kd = s_pos_kd;
  }
}

void Servo_SetCtrlMode(uint8_t mode)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_ctrl_mode = mode;
  Servo_ClearLoopState();
  __set_PRIMASK(primask);
}

uint8_t Servo_GetCtrlMode(void)
{
  return s_ctrl_mode;
}

void Servo_SetZero(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  Foc_EncoderSetZero(&s_enc);
  s_p_set = 0.0f;
  s_vel_i = 0.0f;
  s_pos_i = 0.0f;
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
  s_v_max = CFG_POS_VMAX_DEFAULT;
  s_t_ff = 0.0f;
  s_t_ref = 0.0f;
  s_vel_i = 0.0f;
  s_pos_i = 0.0f;
  if ((s_ctrl_mode == SERVO_CTRL_MOTION) || (s_ctrl_mode == SERVO_CTRL_POSITION))
  {
    s_cmd_valid = 1U;
  }
  else
  {
    s_cmd_valid = 0U;
  }
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
  out->ctrl_mode = s_ctrl_mode;
}

Foc_Encoder_t *Servo_GetEncoder(void)
{
  return &s_enc;
}

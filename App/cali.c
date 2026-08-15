#include "cali.h"

#include "cali_nv.h"
#include "can.h"
#include "config.h"
#include "debug.h"
#include "encoder.h"
#include "foc_encoder.h"
#include "foc_math.h"
#include "pwm.h"
#include "servo.h"
#include "stm32g4xx_hal.h"

static uint8_t Cali_ReadRawMech(float *out)
{
  uint16_t raw;

  if (Encoder_ReadFrame(&raw, 0) == 0U)
  {
    return 0U;
  }
  *out = ((float)raw * FOC_TWO_PI) / (float)ENCODER_CPR;
  return 1U;
}

static void Cali_Fail(const char *why)
{
  Dbg_Printf("%s\r\n", why);
  Pwm_DisableOutputs();
  Servo_SetMode(SERVO_FAULT);
}

static float Cali_Absf(float x)
{
  return (x < 0.0f) ? -x : x;
}

static uint8_t Cali_RotateMeasure(float *delta_mech_out)
{
  float theta;
  float last;
  float acc = 0.0f;
  uint32_t t0;

  if (Cali_ReadRawMech(&last) == 0U)
  {
    return 0U;
  }

  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, CFG_CALI_ROTATE_ELEC_RAD_S, 0.0f);
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < CFG_CALI_ROTATE_MS)
  {
    HAL_Delay(CFG_CALI_ROTATE_SAMPLE_MS);
    if (Cali_ReadRawMech(&theta) == 0U)
    {
      return 0U;
    }
    acc += Foc_WrapAngleToPi(theta - last);
    last = theta;
  }

  *delta_mech_out = acc;
  return 1U;
}

static uint8_t Cali_EstimatePolePairs(float delta_mech, uint8_t *pp_out)
{
  const float elec_delta = CFG_CALI_ROTATE_ELEC_RAD_S *
                           ((float)CFG_CALI_ROTATE_MS / 1000.0f);
  const float abs_mech = Cali_Absf(delta_mech);
  float pp_f;
  uint8_t pp;

  if (abs_mech < CFG_CALI_MIN_MECH_DELTA)
  {
    return 0U;
  }

  pp_f = elec_delta / abs_mech;
  pp = (uint8_t)(pp_f + 0.5f);
  if ((pp < CFG_CALI_PP_MIN) || (pp > CFG_CALI_PP_MAX))
  {
    return 0U;
  }
  if (Cali_Absf(pp_f - (float)pp) > CFG_CALI_PP_MAX_RESIDUAL)
  {
    return 0U;
  }

  *pp_out = pp;
  return 1U;
}

static uint8_t Cali_Run(void)
{
  float theta0;
  float delta;
  float aligned;
  float offset;
  float vel;
  uint8_t pole_pairs = (uint8_t)CFG_POLE_PAIRS;
  int8_t encoder_dir = 1;
  int8_t closed_loop_dir = 1;

  Pwm_EnableOutputs();
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0) == 0U)
  {
    Cali_Fail("cali: enc fail");
    return 0U;
  }

  if (Cali_RotateMeasure(&delta) == 0U)
  {
    Cali_Fail("cali: enc fail");
    return 0U;
  }

  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(80U);

  if ((delta > -CFG_CALI_MIN_MECH_DELTA) && (delta < CFG_CALI_MIN_MECH_DELTA))
  {
    Cali_Fail("cali: no motion");
    return 0U;
  }

  if (delta < 0.0f)
  {
    encoder_dir = -1;
  }

  if (Cali_EstimatePolePairs(delta, &pole_pairs) == 0U)
  {
    Cali_Fail("cali: pp bad");
    return 0U;
  }

  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0) == 0U)
  {
    Cali_Fail("cali: enc fail");
    return 0U;
  }

  aligned = Foc_ApplyEncoderDirToMechTheta(theta0, encoder_dir);
  offset = Foc_WrapAngle0To2Pi(-((float)pole_pairs * aligned));
  Servo_SetPolePairs(pole_pairs);
  Servo_SetEncoderAlignment(encoder_dir, offset);
  Foc_EncoderReset(Servo_GetEncoder(), theta0);

  Servo_SetClosedLoopDir(1);
  Servo_SetVoltageCmd(0.0f, CFG_CALI_PROBE_VQ);
  HAL_Delay(CFG_CALI_PROBE_MS);
  vel = Foc_EncoderGetVelocity(Servo_GetEncoder());
  Servo_SetVoltageCmd(0.0f, 0.0f);
  HAL_Delay(80U);

  if ((vel > -CFG_CALI_MIN_VEL) && (vel < CFG_CALI_MIN_VEL))
  {
    Cali_Fail("cali: probe vel small");
    return 0U;
  }

  if (vel < 0.0f)
  {
    closed_loop_dir = -1;
  }
  Servo_SetClosedLoopDir(closed_loop_dir);
  Servo_HoldPosition();
  Servo_SetMode(SERVO_RUN);
  CtrlTimer_Start();

  Dbg_Printf("cali: ok pp=%d enc_dir=%d cl_dir=%d\r\n",
             (int)pole_pairs,
             (int)encoder_dir,
             (int)closed_loop_dir);
  return 1U;
}

static uint8_t Cali_ApplySaved(const CaliNvData_t *nv)
{
  Foc_Encoder_t *enc = Servo_GetEncoder();
  uint32_t t0;
  float raw_mech;

  Servo_SetPolePairs(nv->pole_pairs);
  Servo_SetEncoderAlignment(nv->encoder_dir, nv->electrical_offset_rad);
  Servo_SetClosedLoopDir(nv->closed_loop_dir);

  t0 = HAL_GetTick();
  while ((Foc_EncoderIsReady(enc) == 0U) && ((HAL_GetTick() - t0) < 50U))
  {
    HAL_Delay(1U);
  }
  if (Foc_EncoderIsReady(enc) == 0U)
  {
    Cali_Fail("cali: enc fail");
    return 0U;
  }

  raw_mech = Foc_EncoderGetLastRaw(enc);
  Foc_EncoderReset(enc, raw_mech);
  Pwm_EnableOutputs();
  Servo_HoldPosition();
  Servo_SetMode(SERVO_RUN);
  CtrlTimer_Start();

  Dbg_Printf("cali: load pp=%d\r\n", (int)nv->pole_pairs);
  return 1U;
}

static void Cali_ResumeRun(void)
{
  Can_Restart();
  Encoder_Init();
  CtrlTimer_Start();
  Pwm_EnableOutputs();
  Servo_HoldPosition();
  Servo_SetMode(SERVO_RUN);
}

static void Cali_SaveCurrent(void)
{
  CaliNvData_t nv;
  ServoTelemetry_t tel;
  const Foc_Encoder_t *enc = Servo_GetEncoder();

  Servo_GetTelemetry(&tel);
  nv.pole_pairs = enc->pole_pairs;
  nv.encoder_dir = enc->direction;
  nv.closed_loop_dir = tel.closed_loop_dir;
  nv.electrical_offset_rad = enc->electrical_offset_rad;

  Servo_SetMode(SERVO_IDLE);
  Pwm_ApplyDuty(FOC_PWM_NEUTRAL_DUTY, FOC_PWM_NEUTRAL_DUTY, FOC_PWM_NEUTRAL_DUTY);
  Pwm_DisableOutputs();
  Can_StopForFlash();

  if (CaliNv_Save(&nv) == 0U)
  {
    Dbg_Printf("cali: save fail\r\n");
    Cali_ResumeRun();
    return;
  }

  Cali_ResumeRun();
  Dbg_Printf("cali: saved pp=%d\r\n", (int)nv.pole_pairs);
}

uint8_t Cali_Start(void)
{
  CaliNvData_t nv;

  if (CaliNv_Load(&nv) != 0U)
  {
    return Cali_ApplySaved(&nv);
  }

  Dbg_Printf("cali: no nv, run\r\n");
  Can_StopForFlash();
  if (Cali_Run() == 0U)
  {
    Can_Restart();
    return 0U;
  }
  Cali_SaveCurrent();
  return 1U;
}

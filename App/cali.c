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

#include <math.h>

#define CALI_ST_IDLE         (0x00U)
#define CALI_ST_RUNNING      (0x01U)
#define CALI_ST_SUCCESS      (0x02U)
#define CALI_ST_FAILED       (0x03U)

#define CALI_STAGE_IDLE      (0x00U)
#define CALI_STAGE_PRE_CHECK (0x01U)
#define CALI_STAGE_ALIGN     (0x02U)
#define CALI_STAGE_SAMPLE    (0x03U)
#define CALI_STAGE_CALC      (0x04U)
#define CALI_STAGE_SAVE      (0x05U)
#define CALI_STAGE_COMPLETE  (0x06U)

#define CALI_ERR_NONE        (0x0000U)
#define CALI_ERR_ENCODER     (0x0002U)
#define CALI_ERR_DATA        (0x0004U)
#define CALI_ERR_SAVE        (0x0005U)

static uint8_t s_rpt_en;
static uint8_t s_rpt_seq;
static uint8_t s_rpt_state;
static uint8_t s_rpt_progress;
static uint8_t s_rpt_stage;
static uint16_t s_rpt_error;
static uint32_t s_rpt_last_ms;

static void Cali_SendReport(void)
{
  uint8_t data[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

  if (s_rpt_en == 0U)
  {
    return;
  }

  data[0] = s_rpt_seq;
  data[1] = s_rpt_state;
  data[2] = s_rpt_progress;
  data[3] = s_rpt_stage;
  data[4] = (uint8_t)(s_rpt_error & 0xFFU);
  data[5] = (uint8_t)(s_rpt_error >> 8);
  (void)Can_Send(CFG_CAN_CALI_RPT_BASE + CFG_NODE_ID, data);
  s_rpt_last_ms = HAL_GetTick();
}

static void Cali_SetReport(uint8_t state, uint8_t progress, uint8_t stage, uint16_t err)
{
  const uint8_t stage_changed = (stage != s_rpt_stage) ? 1U : 0U;

  s_rpt_state = state;
  s_rpt_progress = progress;
  s_rpt_stage = stage;
  s_rpt_error = err;
  if ((s_rpt_en != 0U) &&
      ((stage_changed != 0U) ||
       ((HAL_GetTick() - s_rpt_last_ms) >= CFG_CALI_REPORT_MS)))
  {
    Cali_SendReport();
  }
}

static void Cali_Delay(uint32_t ms)
{
  const uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < ms)
  {
    if (s_rpt_en != 0U)
    {
      if ((HAL_GetTick() - s_rpt_last_ms) >= CFG_CALI_REPORT_MS)
      {
        Cali_SendReport();
      }
      Can_Service();
    }
    HAL_Delay(1U);
  }
}

static uint8_t Cali_ReadRawMech(float *out)
{
  const Foc_Encoder_t *enc = Servo_GetEncoder();
  uint32_t t0;

  if (Foc_EncoderIsReady(enc) != 0U)
  {
    *out = Foc_EncoderGetLastRaw(enc);
    return 1U;
  }

  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 20U)
  {
    Cali_Delay(1U);
    if (Foc_EncoderIsReady(enc) != 0U)
    {
      *out = Foc_EncoderGetLastRaw(enc);
      return 1U;
    }
  }
  return 0U;
}

static void Cali_Fail(const char *why, uint16_t err)
{
  Dbg_Printf("%s\r\n", why);
  Cali_SetReport(CALI_ST_FAILED, s_rpt_progress, s_rpt_stage, err);
  if (s_rpt_en != 0U)
  {
    Cali_SendReport();
  }
  Pwm_DisableOutputs();
  Servo_SetMode(SERVO_FAULT);
}

static float Cali_Absf(float x)
{
  return (x < 0.0f) ? -x : x;
}

typedef struct
{
  float sin_p;
  float cos_p;
  float sin_n;
  float cos_n;
  uint32_t n;
} CaliOffsetAcc_t;

static void Cali_OffsetAccAdd(CaliOffsetAcc_t *acc, float elec, float raw_mech)
{
  float s;
  float c;
  float off;
  float aligned;
  const float pp = (float)CFG_POLE_PAIRS;

  aligned = Foc_ApplyEncoderDirToMechTheta(raw_mech, 1);
  off = Foc_WrapAngleToPi(elec - (pp * aligned));
  Foc_SinCos(off, &s, &c);
  acc->sin_p += s;
  acc->cos_p += c;

  aligned = Foc_ApplyEncoderDirToMechTheta(raw_mech, -1);
  off = Foc_WrapAngleToPi(elec - (pp * aligned));
  Foc_SinCos(off, &s, &c);
  acc->sin_n += s;
  acc->cos_n += c;
  acc->n++;
}

static uint8_t Cali_RotateMeasure(float elec_rate, float *delta_mech_out, CaliOffsetAcc_t *off)
{
  float theta;
  float last;
  float acc = 0.0f;
  uint32_t t0;
  uint32_t miss = 0U;

  if (Cali_ReadRawMech(&last) == 0U)
  {
    return 0U;
  }

  Servo_SetOpenloopRate(elec_rate);
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < CFG_CALI_ROTATE_MS)
  {
    Cali_Delay(CFG_CALI_ROTATE_SAMPLE_MS);
    if (Cali_ReadRawMech(&theta) == 0U)
    {
      miss++;
      if (miss > 20U)
      {
        Servo_SetOpenloopRate(0.0f);
        return 0U;
      }
      continue;
    }
    miss = 0U;
    acc += Foc_WrapAngleToPi(theta - last);
    last = theta;
    if ((off != 0) && ((HAL_GetTick() - t0) >= CFG_CALI_OFFSET_SKIP_MS))
    {
      Cali_OffsetAccAdd(off, Servo_GetOpenloopElec(), theta);
    }
  }

  Servo_SetOpenloopRate(0.0f);
  *delta_mech_out = acc;
  return 1U;
}

static uint8_t Cali_EstimatePolePairs(float abs_mech, float elec_abs, uint8_t *pp_out)
{
  float pp_f;
  uint8_t pp;

  if ((abs_mech < CFG_CALI_MIN_MECH_DELTA) || (elec_abs < 1.0e-6f))
  {
    return 0U;
  }

  pp_f = elec_abs / abs_mech;
  if (Cali_Absf(pp_f - (float)CFG_POLE_PAIRS) <= CFG_CALI_PP_MAX_RESIDUAL)
  {
    *pp_out = (uint8_t)CFG_POLE_PAIRS;
    return 1U;
  }

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
  CaliOffsetAcc_t offacc;
  float theta0;
  float d_fwd;
  float d_rev;
  float aligned;
  float offset;
  float vel;
  uint8_t pole_pairs = (uint8_t)CFG_POLE_PAIRS;
  int8_t encoder_dir = 1;
  int8_t closed_loop_dir = 1;

  offacc.sin_p = 0.0f;
  offacc.cos_p = 0.0f;
  offacc.sin_n = 0.0f;
  offacc.cos_n = 0.0f;
  offacc.n = 0U;

  Cali_SetReport(CALI_ST_RUNNING, 10U, CALI_STAGE_ALIGN, CALI_ERR_NONE);
  Pwm_EnableOutputs();
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  Cali_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0) == 0U)
  {
    Cali_Fail("cali: enc fail", CALI_ERR_ENCODER);
    return 0U;
  }
  {
    uint16_t raw0 = 0U;
    uint16_t raw1 = 0U;
    (void)Encoder_ReadFrame(&raw0, 0);
    Cali_Delay(20U);
    (void)Encoder_ReadFrame(&raw1, 0);
    Dbg_Printf("cali: raw=%u %u th0=%d\r\n",
               (unsigned)raw0,
               (unsigned)raw1,
               (int)(theta0 * 1000.0f));
  }

  Cali_SetReport(CALI_ST_RUNNING, 25U, CALI_STAGE_SAMPLE, CALI_ERR_NONE);
  if (Cali_RotateMeasure(CFG_CALI_ROTATE_ELEC_RAD_S, &d_fwd, &offacc) == 0U)
  {
    Cali_Fail("cali: enc fail", CALI_ERR_ENCODER);
    return 0U;
  }
  Cali_Delay(80U);

  Cali_SetReport(CALI_ST_RUNNING, 40U, CALI_STAGE_SAMPLE, CALI_ERR_NONE);
  if (Cali_RotateMeasure(-CFG_CALI_ROTATE_ELEC_RAD_S, &d_rev, &offacc) == 0U)
  {
    Cali_Fail("cali: enc fail", CALI_ERR_ENCODER);
    return 0U;
  }
  Cali_Delay(80U);

  if ((Cali_Absf(d_fwd) < CFG_CALI_MIN_MECH_DELTA) ||
      (Cali_Absf(d_rev) < CFG_CALI_MIN_MECH_DELTA))
  {
    Cali_Fail("cali: no motion", CALI_ERR_DATA);
    return 0U;
  }
  if ((d_fwd * d_rev) > 0.0f)
  {
    Cali_Fail("cali: dir mismatch", CALI_ERR_DATA);
    return 0U;
  }

  if (d_fwd < 0.0f)
  {
    encoder_dir = -1;
  }

  {
    const float elec_one = CFG_CALI_ROTATE_ELEC_RAD_S *
                           ((float)CFG_CALI_ROTATE_MS / 1000.0f);
    const float abs_avg = 0.5f * (Cali_Absf(d_fwd) + Cali_Absf(d_rev));
    const float pp_f = (abs_avg > 1.0e-6f) ? (elec_one / abs_avg) : 0.0f;
    Dbg_Printf("cali: df=%d dr=%d ppx100=%d\r\n",
               (int)(d_fwd * 1000.0f),
               (int)(d_rev * 1000.0f),
               (int)(pp_f * 100.0f));
    if (Cali_EstimatePolePairs(abs_avg, elec_one, &pole_pairs) == 0U)
    {
      Cali_Fail("cali: pp bad", CALI_ERR_DATA);
      return 0U;
    }
  }

  Cali_SetReport(CALI_ST_RUNNING, 60U, CALI_STAGE_CALC, CALI_ERR_NONE);
  Servo_SetOpenloopRate(0.0f);
  Cali_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0) == 0U)
  {
    Cali_Fail("cali: enc fail", CALI_ERR_ENCODER);
    return 0U;
  }

  aligned = Foc_ApplyEncoderDirToMechTheta(theta0, encoder_dir);
  if ((offacc.n > 8U) && (pole_pairs == (uint8_t)CFG_POLE_PAIRS))
  {
    if (encoder_dir < 0)
    {
      offset = Foc_WrapAngle0To2Pi(atan2f(offacc.sin_n, offacc.cos_n));
    }
    else
    {
      offset = Foc_WrapAngle0To2Pi(atan2f(offacc.sin_p, offacc.cos_p));
    }
  }
  else
  {
    offset = Foc_WrapAngle0To2Pi(-((float)pole_pairs * aligned));
  }
  Dbg_Printf("cali: off=%d n=%u\r\n", (int)(offset * 1000.0f), (unsigned)offacc.n);
  Servo_SetPolePairs(pole_pairs);
  Servo_SetEncoderAlignment(encoder_dir, offset);
  Foc_EncoderReset(Servo_GetEncoder(), theta0);

  Servo_SetClosedLoopDir(1);
  {
    float last;
    float theta;
    float acc = 0.0f;
    uint32_t t0;

    if (Cali_ReadRawMech(&last) == 0U)
    {
      Cali_Fail("cali: enc fail", CALI_ERR_ENCODER);
      return 0U;
    }
    last = Foc_ApplyEncoderDirToMechTheta(last, encoder_dir);
    Servo_SetVoltageCmd(0.0f, CFG_CALI_PROBE_VQ);
    t0 = HAL_GetTick();
    {
      uint32_t miss = 0U;

      while ((HAL_GetTick() - t0) < CFG_CALI_PROBE_MS)
      {
        Cali_Delay(CFG_CALI_ROTATE_SAMPLE_MS);
        if (Cali_ReadRawMech(&theta) == 0U)
        {
          miss++;
          if (miss > 20U)
          {
            Servo_SetVoltageCmd(0.0f, 0.0f);
            Cali_Fail("cali: enc fail", CALI_ERR_ENCODER);
            return 0U;
          }
          continue;
        }
        miss = 0U;
        theta = Foc_ApplyEncoderDirToMechTheta(theta, encoder_dir);
        acc += Foc_WrapAngleToPi(theta - last);
        last = theta;
      }
    }
    vel = Foc_EncoderGetVelocity(Servo_GetEncoder());
    Servo_SetVoltageCmd(0.0f, 0.0f);
    Cali_Delay(80U);
    Dbg_Printf("cali: probe d=%d v=%d\r\n", (int)(acc * 1000.0f), (int)(vel * 1000.0f));
    if ((acc > -CFG_CALI_MIN_MECH_DELTA) && (acc < CFG_CALI_MIN_MECH_DELTA))
    {
      Cali_Fail("cali: probe vel small", CALI_ERR_DATA);
      return 0U;
    }
    if (acc < 0.0f)
    {
      closed_loop_dir = -1;
    }
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
    Cali_Fail("cali: enc fail", CALI_ERR_ENCODER);
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

static uint8_t Cali_SaveCurrent(void)
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
    Can_Restart();
    return 0U;
  }

  Cali_ResumeRun();
  Dbg_Printf("cali: saved pp=%d\r\n", (int)nv.pole_pairs);
  return 1U;
}

uint8_t Cali_RunCommand(uint8_t seq)
{
  s_rpt_en = 1U;
  s_rpt_seq = seq;
  s_rpt_state = CALI_ST_RUNNING;
  s_rpt_progress = 5U;
  s_rpt_stage = CALI_STAGE_PRE_CHECK;
  s_rpt_error = CALI_ERR_NONE;
  s_rpt_last_ms = 0U;
  Cali_SendReport();

  Servo_SetMode(SERVO_IDLE);
  Pwm_ApplyDuty(FOC_PWM_NEUTRAL_DUTY, FOC_PWM_NEUTRAL_DUTY, FOC_PWM_NEUTRAL_DUTY);
  Servo_HoldPosition();

  if (Cali_Run() == 0U)
  {
    s_rpt_en = 0U;
    return 0U;
  }

  Cali_SetReport(CALI_ST_RUNNING, 90U, CALI_STAGE_SAVE, CALI_ERR_NONE);
  Cali_SendReport();
  if (Cali_SaveCurrent() == 0U)
  {
    Cali_Fail("cali: save fail", CALI_ERR_SAVE);
    s_rpt_en = 0U;
    return 0U;
  }

  s_rpt_progress = 100U;
  s_rpt_stage = CALI_STAGE_COMPLETE;
  s_rpt_state = CALI_ST_SUCCESS;
  s_rpt_error = CALI_ERR_NONE;
  Cali_SendReport();
  s_rpt_en = 0U;
  return 1U;
}

uint8_t Cali_Start(void)
{
  CaliNvData_t nv;

  if (CaliNv_Load(&nv) != 0U)
  {
    return Cali_ApplySaved(&nv);
  }

  Dbg_Printf("cali: no nv, run\r\n");
  return Cali_RunCommand(0U);
}

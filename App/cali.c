#include "cali.h"

#include "config.h"
#include "debug.h"
#include "encoder.h"
#include "foc_math.h"
#include "pwm.h"
#include "servo.h"
#include "stm32g4xx_hal.h"

static uint8_t Cali_ReadRawMech(float *out, uint16_t *raw_out, uint8_t *status_out)
{
  uint16_t raw;
  uint8_t status = 0U;

  if (Encoder_ReadFrame(&raw, &status) == 0U)
  {
    return 0U;
  }
  *out = ((float)raw * FOC_TWO_PI) / (float)ENCODER_CPR;
  if (raw_out != 0)
  {
    *raw_out = raw;
  }
  if (status_out != 0)
  {
    *status_out = status;
  }
  return 1U;
}

static void Cali_Fail(const char *why)
{
  Dbg_Printf("%s\r\n", why);
  Pwm_DisableOutputs();
  Servo_SetMode(SERVO_FAULT);
}

uint8_t Cali_Run(void)
{
  float theta0;
  float theta1;
  float delta;
  float aligned;
  float offset;
  float vel;
  uint16_t raw0;
  uint16_t raw1;
  uint8_t st0;
  uint8_t st1;
  int8_t encoder_dir = 1;
  int8_t closed_loop_dir = 1;

  Dbg_Printf("cali: lock\r\n");
  Pwm_EnableOutputs();
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0, &raw0, &st0) == 0U)
  {
    Cali_Fail("cali: enc fail");
    return 0U;
  }
  Dbg_Printf("cali: lock raw=%d st=%d\r\n", (int)raw0, (int)st0);

  Dbg_Printf("cali: rotate\r\n");
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, CFG_CALI_ROTATE_ELEC_RAD_S, 0.0f);
  HAL_Delay(CFG_CALI_ROTATE_MS);
  if (Cali_ReadRawMech(&theta1, &raw1, &st1) == 0U)
  {
    Cali_Fail("cali: enc fail");
    return 0U;
  }

  delta = Foc_WrapAngleToPi(theta1 - theta0);
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(80U);

  Dbg_Printf("cali: rot d_mrad=%d raw=%d st=%d\r\n",
             (int)(delta * 1000.0f),
             (int)raw1,
             (int)st1);

  if ((delta > -CFG_CALI_MIN_MECH_DELTA) && (delta < CFG_CALI_MIN_MECH_DELTA))
  {
    Cali_Fail("cali: no motion");
    return 0U;
  }

  if (delta < 0.0f)
  {
    encoder_dir = -1;
  }

  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0, &raw0, &st0) == 0U)
  {
    Cali_Fail("cali: enc fail");
    return 0U;
  }

  aligned = Foc_ApplyEncoderDirToMechTheta(theta0, encoder_dir);
  offset = Foc_WrapAngle0To2Pi(-((float)CFG_POLE_PAIRS * aligned));
  Servo_SetEncoderAlignment(encoder_dir, offset);
  Foc_EncoderReset(Servo_GetEncoder(), theta0);

  Dbg_Printf("cali: probe enc_dir=%d off_mrad=%d raw=%d st=%d\r\n",
             (int)encoder_dir,
             (int)(offset * 1000.0f),
             (int)raw0,
             (int)st0);

  Servo_SetClosedLoopDir(1);
  Servo_SetVoltageCmd(0.0f, CFG_CALI_PROBE_VQ);
  HAL_Delay(CFG_CALI_PROBE_MS);
  vel = Foc_EncoderGetVelocity(Servo_GetEncoder());
  Servo_SetVoltageCmd(0.0f, 0.0f);
  HAL_Delay(80U);

  Dbg_Printf("cali: probe vel_mrad_s=%d\r\n", (int)(vel * 1000.0f));

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
  Servo_ZeroPosition();
  Servo_HoldPosition();
  Servo_SetMode(SERVO_RUN);
  CtrlTimer_Start();

  Dbg_Printf("cali: ok enc_dir=%d cl_dir=%d\r\n",
             (int)encoder_dir,
             (int)closed_loop_dir);
  return 1U;
}

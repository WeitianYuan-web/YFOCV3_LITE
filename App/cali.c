#include "cali.h"

#include "config.h"
#include "debug.h"
#include "encoder.h"
#include "foc_math.h"
#include "pwm.h"
#include "servo.h"
#include "stm32g4xx_hal.h"

static uint8_t Cali_ReadRawMech(float *out)
{
  uint16_t raw;

  if (Encoder_ReadRawFast(&raw) == 0U)
  {
    return 0U;
  }
  *out = ((float)raw * FOC_TWO_PI) / (float)ENCODER_CPR;
  return 1U;
}

uint8_t Cali_Run(void)
{
  float theta0;
  float theta1;
  float delta;
  float aligned;
  float offset;
  float vel;
  int8_t encoder_dir = 1;
  int8_t closed_loop_dir = 1;

  Dbg_Printf("cali: lock\r\n");
  Pwm_EnableOutputs();
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0) == 0U)
  {
    Dbg_Printf("cali: enc fail\r\n");
    Pwm_DisableOutputs();
    Servo_SetMode(SERVO_FAULT);
    return 0U;
  }

  Dbg_Printf("cali: rotate\r\n");
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, CFG_CALI_ROTATE_ELEC_RAD_S, 0.0f);
  HAL_Delay(CFG_CALI_ROTATE_MS);
  if (Cali_ReadRawMech(&theta1) == 0U)
  {
    Dbg_Printf("cali: enc fail\r\n");
    Pwm_DisableOutputs();
    Servo_SetMode(SERVO_FAULT);
    return 0U;
  }

  delta = Foc_WrapAngleToPi(theta1 - theta0);
  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(80U);

  if ((delta > -CFG_CALI_MIN_MECH_DELTA) && (delta < CFG_CALI_MIN_MECH_DELTA))
  {
    Dbg_Printf("cali: no motion\r\n");
    Pwm_DisableOutputs();
    Servo_SetMode(SERVO_FAULT);
    return 0U;
  }

  if (delta < 0.0f)
  {
    encoder_dir = -1;
  }

  Servo_SetOpenloop(CFG_CALI_LOCK_V, 0.0f, 0.0f, 0.0f);
  HAL_Delay(CFG_CALI_LOCK_MS);
  if (Cali_ReadRawMech(&theta0) == 0U)
  {
    Dbg_Printf("cali: enc fail\r\n");
    Pwm_DisableOutputs();
    Servo_SetMode(SERVO_FAULT);
    return 0U;
  }

  aligned = Foc_ApplyEncoderDirToMechTheta(theta0, encoder_dir);
  offset = Foc_WrapAngle0To2Pi(-((float)CFG_POLE_PAIRS * aligned));
  Servo_SetEncoderAlignment(encoder_dir, offset);
  Foc_EncoderReset(Servo_GetEncoder(), theta0);

  Dbg_Printf("cali: probe enc_dir=%d off_mrad=%d\r\n",
             (int)encoder_dir,
             (int)(offset * 1000.0f));

  Servo_SetClosedLoopDir(1);
  Servo_SetVoltageCmd(0.0f, CFG_CALI_PROBE_VQ);
  HAL_Delay(CFG_CALI_PROBE_MS);
  vel = Foc_EncoderGetVelocity(Servo_GetEncoder());
  Servo_SetVoltageCmd(0.0f, 0.0f);
  HAL_Delay(80U);

  if ((vel > -CFG_CALI_MIN_VEL) && (vel < CFG_CALI_MIN_VEL))
  {
    Dbg_Printf("cali: probe vel small\r\n");
    Pwm_DisableOutputs();
    Servo_SetMode(SERVO_FAULT);
    return 0U;
  }

  if (vel < 0.0f)
  {
    closed_loop_dir = -1;
  }
  Servo_SetClosedLoopDir(closed_loop_dir);
  Servo_ZeroPosition();
  Servo_SetMode(SERVO_RUN);
  CtrlTimer_Start();

  Dbg_Printf("cali: ok enc_dir=%d cl_dir=%d\r\n",
             (int)encoder_dir,
             (int)closed_loop_dir);
  return 1U;
}

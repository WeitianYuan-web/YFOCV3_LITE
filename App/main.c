#include "board.h"
#include "cali.h"
#include "can.h"
#include "comm.h"
#include "config.h"
#include "debug.h"
#include "encoder.h"
#include "pwm.h"
#include "servo.h"

int main(void)
{
  uint32_t last_dbg_ms = 0U;
  uint32_t last_fb_ms = 0U;
  uint8_t cali_ok;

  Board_Init();
  Dbg_Init();
  Encoder_Init();
  Servo_Init();
  Can_Init((uint8_t)CFG_NODE_ID);
  Pwm_StartTimer();

  Dbg_Printf("YFOCV3 LC-ESC voltage servo\r\n");
  Board_LedSet(1U);
  cali_ok = Cali_Run();
  if (cali_ok == 0U)
  {
    Dbg_Printf("cali failed, halt\r\n");
    for (;;)
    {
      Board_LedToggle();
      HAL_Delay(80U);
    }
  }

  Board_LedSet(1U);
  Dbg_Printf("run\r\n");

  for (;;)
  {
    uint32_t now;

    Can_Service();
    Comm_Process();

    now = HAL_GetTick();
    if ((now - last_fb_ms) >= CFG_FB_PERIOD_MS)
    {
      last_fb_ms = now;
      Comm_SendFeedback();
    }
    if ((now - last_dbg_ms) >= CFG_DEBUG_PERIOD_MS)
    {
      ServoTelemetry_t tel;
      last_dbg_ms = now;
      Servo_GetTelemetry(&tel);
      Dbg_Printf("p=%d v=%d t=%d kp=%d kd=%d\r\n",
                 (int)(tel.p_act * 1000.0f),
                 (int)(tel.v_act * 1000.0f),
                 (int)(tel.t_ref * 1000.0f),
                 (int)(tel.kp * 10.0f),
                 (int)(tel.kd * 1000.0f));
    }
  }
}

#include "comm.h"

#include "can.h"
#include "config.h"
#include "foc_math.h"
#include "servo.h"

static uint16_t Comm_ReadU16Be(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void Comm_WriteU16Be(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFFU);
}

static float Comm_DecodeU16(uint16_t raw, float min_v, float max_v)
{
  return min_v + ((float)raw * (max_v - min_v) / 65535.0f);
}

static uint16_t Comm_EncodeU16(float x, float min_v, float max_v)
{
  float t = (x - min_v) / (max_v - min_v);
  t = Foc_Clamp(t, 0.0f, 1.0f);
  return (uint16_t)((t * 65535.0f) + 0.5f);
}

void Comm_Process(void)
{
  CanFrame_t frame;
  const uint32_t cmd_id = CFG_CAN_CMD_BASE + CFG_NODE_ID;

  while (Can_PopRx(&frame) != 0U)
  {
    float p_set;
    float v_set;
    float kp;
    float kd;

    if (frame.id != cmd_id)
    {
      continue;
    }

    p_set = Comm_DecodeU16(Comm_ReadU16Be(&frame.data[0]), CFG_POS_CMD_MIN, CFG_POS_CMD_MAX);
    v_set = Comm_DecodeU16(Comm_ReadU16Be(&frame.data[2]), CFG_VEL_CMD_MIN, CFG_VEL_CMD_MAX);
    kp = Comm_DecodeU16(Comm_ReadU16Be(&frame.data[4]), CFG_KP_MIN, CFG_KP_MAX);
    kd = Comm_DecodeU16(Comm_ReadU16Be(&frame.data[6]), CFG_KD_MIN, CFG_KD_MAX);
    Servo_SetCommand(p_set, v_set, kp, kd);
  }
}

void Comm_SendFeedback(void)
{
  ServoTelemetry_t tel;
  uint8_t data[8];
  float pos_wrap;
  float turns_f;
  uint16_t turns;
  const uint32_t fb_id = CFG_CAN_FB_BASE + CFG_NODE_ID;

  Servo_GetTelemetry(&tel);
  pos_wrap = Foc_WrapAngleToPi(tel.p_act);
  turns_f = tel.p_act / FOC_TWO_PI;
  if (turns_f < 0.0f)
  {
    turns_f += 65536.0f;
  }
  turns = (uint16_t)turns_f;

  Comm_WriteU16Be(&data[0], Comm_EncodeU16(pos_wrap, CFG_POS_FB_MIN, CFG_POS_FB_MAX));
  Comm_WriteU16Be(&data[2], Comm_EncodeU16(tel.v_act, CFG_VEL_CMD_MIN, CFG_VEL_CMD_MAX));
  Comm_WriteU16Be(&data[4], Comm_EncodeU16(tel.t_ref, CFG_TORQUE_FB_MIN, CFG_TORQUE_FB_MAX));
  Comm_WriteU16Be(&data[6], turns);
  (void)Can_Send(fb_id, data);
}

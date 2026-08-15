#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "foc_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  SERVO_IDLE = 0,
  SERVO_OPENLOOP,
  SERVO_VOLTAGE,
  SERVO_RUN,
  SERVO_FAULT
} ServoMode_t;

typedef struct
{
  float p_set;
  float v_set;
  float kp;
  float kd;
  float t_ref;
  float p_act;
  float v_act;
  float elec;
  int8_t closed_loop_dir;
} ServoTelemetry_t;

void Servo_Init(void);
void Servo_OnPwmIsr(void);
void Servo_OnCtrlIsr(void);

void Servo_SetMode(ServoMode_t mode);
ServoMode_t Servo_GetMode(void);
void Servo_SetOpenloop(float d_v, float q_v, float elec_rate_rad_s, float elec_angle_rad);
void Servo_SetVoltageCmd(float d_v, float q_v);
void Servo_SetCommand(float p_set, float v_set, float kp, float kd);
void Servo_SetClosedLoopDir(int8_t dir);
void Servo_SetEncoderAlignment(int8_t encoder_dir, float electrical_offset_rad);
void Servo_SetPolePairs(uint8_t pole_pairs);
void Servo_HoldPosition(void);
void Servo_GetTelemetry(ServoTelemetry_t *out);
Foc_Encoder_t *Servo_GetEncoder(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */

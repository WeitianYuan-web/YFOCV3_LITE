#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include <stdint.h>
#include "foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t pole_pairs;
  int8_t  direction;
  float   electrical_offset_rad;
  float   vel_lpf_hz;
  float   sample_hz;
} Foc_EncoderConfig_t;

typedef struct
{
  uint8_t pole_pairs;
  int8_t  direction;
  float   electrical_offset_rad;
  uint8_t initialized;
  float   last_raw_mech;
  float   mech_wrapped;
  float   mech_unwrapped;
  float   mech_zero;
  float   last_elec;
  float   vel_hat;
  float   elec_angle;
  float   lpf_b0;
  float   lpf_b1;
  float   lpf_b2;
  float   lpf_a1;
  float   lpf_a2;
  float   lpf_x1;
  float   lpf_x2;
  float   lpf_y1;
  float   lpf_y2;
} Foc_Encoder_t;

void Foc_EncoderInit(Foc_Encoder_t *encoder, const Foc_EncoderConfig_t *config);
void Foc_EncoderReset(Foc_Encoder_t *encoder, float raw_mech_angle_rad);
void Foc_EncoderSetZero(Foc_Encoder_t *encoder);
uint8_t Foc_EncoderUpdate(Foc_Encoder_t *encoder, float raw_mech_angle_rad, float dt_s);
void Foc_EncoderPredict(Foc_Encoder_t *encoder, float dt_s);
void Foc_EncoderSetAlignment(Foc_Encoder_t *encoder, int8_t direction, float electrical_offset_rad);
void Foc_EncoderSetPolePairs(Foc_Encoder_t *encoder, uint8_t pole_pairs);

float Foc_EncoderGetMechUnwrapped(const Foc_Encoder_t *encoder);
float Foc_EncoderGetPosition(const Foc_Encoder_t *encoder);
float Foc_EncoderGetVelocity(const Foc_Encoder_t *encoder);
float Foc_EncoderGetElectrical(const Foc_Encoder_t *encoder);
float Foc_EncoderGetMechWrapped(const Foc_Encoder_t *encoder);
float Foc_EncoderGetLastRaw(const Foc_Encoder_t *encoder);
uint8_t Foc_EncoderIsReady(const Foc_Encoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ENCODER_H */

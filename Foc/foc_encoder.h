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
  float   lpf_alpha;
} Foc_EncoderConfig_t;

typedef struct
{
  uint8_t pole_pairs;
  int8_t  direction;
  float   electrical_offset_rad;
  float   lpf_alpha;
  uint8_t initialized;
  float   last_raw_mech;
  float   mech_wrapped;
  float   mech_unwrapped;
  float   mech_zero;
  float   vel_lpf;
  float   elec_angle;
} Foc_Encoder_t;

void Foc_EncoderInit(Foc_Encoder_t *encoder, const Foc_EncoderConfig_t *config);
void Foc_EncoderReset(Foc_Encoder_t *encoder, float raw_mech_angle_rad);
void Foc_EncoderSetZero(Foc_Encoder_t *encoder);
uint8_t Foc_EncoderUpdate(Foc_Encoder_t *encoder, float raw_mech_angle_rad, float dt_s);
void Foc_EncoderSetAlignment(Foc_Encoder_t *encoder, int8_t direction, float electrical_offset_rad);

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

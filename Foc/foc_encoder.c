#include "foc_encoder.h"

void Foc_EncoderInit(Foc_Encoder_t *encoder, const Foc_EncoderConfig_t *config)
{
  encoder->pole_pairs = config->pole_pairs;
  encoder->direction = (config->direction < 0) ? -1 : 1;
  encoder->electrical_offset_rad = config->electrical_offset_rad;
  encoder->lpf_alpha = config->lpf_alpha;
  encoder->initialized = 0U;
  encoder->last_raw_mech = 0.0f;
  encoder->mech_wrapped = 0.0f;
  encoder->mech_unwrapped = 0.0f;
  encoder->mech_zero = 0.0f;
  encoder->vel_lpf = 0.0f;
  encoder->elec_angle = 0.0f;
}

void Foc_EncoderSetAlignment(Foc_Encoder_t *encoder, int8_t direction, float electrical_offset_rad)
{
  encoder->direction = (direction < 0) ? -1 : 1;
  encoder->electrical_offset_rad = electrical_offset_rad;
}

void Foc_EncoderReset(Foc_Encoder_t *encoder, float raw_mech_angle_rad)
{
  const float aligned = Foc_ApplyEncoderDirToMechTheta(raw_mech_angle_rad, encoder->direction);
  encoder->last_raw_mech = raw_mech_angle_rad;
  encoder->mech_wrapped = aligned;
  encoder->mech_unwrapped = aligned;
  encoder->mech_zero = aligned;
  encoder->vel_lpf = 0.0f;
  encoder->elec_angle = Foc_WrapAngle0To2Pi(
      ((float)encoder->pole_pairs * aligned) + encoder->electrical_offset_rad);
  encoder->initialized = 1U;
}

void Foc_EncoderSetZero(Foc_Encoder_t *encoder)
{
  encoder->mech_zero = encoder->mech_unwrapped;
}

uint8_t Foc_EncoderUpdate(Foc_Encoder_t *encoder, float raw_mech_angle_rad, float dt_s)
{
  float aligned;
  float delta;
  float vel_raw;

  aligned = Foc_ApplyEncoderDirToMechTheta(raw_mech_angle_rad, encoder->direction);

  if (encoder->initialized == 0U)
  {
    Foc_EncoderReset(encoder, raw_mech_angle_rad);
    return 1U;
  }

  if (dt_s <= 0.0f)
  {
    return 0U;
  }

  delta = Foc_WrapAngleToPi(aligned - encoder->mech_wrapped);
  encoder->mech_wrapped = aligned;
  encoder->mech_unwrapped += delta;

  vel_raw = delta / dt_s;
  encoder->vel_lpf += encoder->lpf_alpha * (vel_raw - encoder->vel_lpf);

  encoder->elec_angle = Foc_WrapAngle0To2Pi(
      ((float)encoder->pole_pairs * aligned) + encoder->electrical_offset_rad);

  return 1U;
}

float Foc_EncoderGetMechUnwrapped(const Foc_Encoder_t *encoder)
{
  return encoder->mech_unwrapped;
}

float Foc_EncoderGetPosition(const Foc_Encoder_t *encoder)
{
  return encoder->mech_unwrapped - encoder->mech_zero;
}

float Foc_EncoderGetVelocity(const Foc_Encoder_t *encoder)
{
  return encoder->vel_lpf;
}

float Foc_EncoderGetElectrical(const Foc_Encoder_t *encoder)
{
  return encoder->elec_angle;
}

float Foc_EncoderGetMechWrapped(const Foc_Encoder_t *encoder)
{
  return encoder->mech_wrapped;
}

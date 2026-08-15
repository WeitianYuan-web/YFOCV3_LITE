#include "foc_encoder.h"

void Foc_EncoderInit(Foc_Encoder_t *encoder, const Foc_EncoderConfig_t *config)
{
  encoder->pole_pairs = config->pole_pairs;
  encoder->direction = (config->direction < 0) ? -1 : 1;
  encoder->electrical_offset_rad = config->electrical_offset_rad;
  encoder->pll_kp = config->pll_kp;
  encoder->pll_ki = config->pll_ki;
  encoder->initialized = 0U;
  encoder->last_raw_mech = 0.0f;
  encoder->mech_wrapped = 0.0f;
  encoder->mech_unwrapped = 0.0f;
  encoder->mech_zero = 0.0f;
  encoder->pll_theta = 0.0f;
  encoder->vel_hat = 0.0f;
  encoder->elec_angle = 0.0f;
}

void Foc_EncoderSetAlignment(Foc_Encoder_t *encoder, int8_t direction, float electrical_offset_rad)
{
  encoder->direction = (direction < 0) ? -1 : 1;
  encoder->electrical_offset_rad = electrical_offset_rad;
}

void Foc_EncoderSetPolePairs(Foc_Encoder_t *encoder, uint8_t pole_pairs)
{
  encoder->pole_pairs = pole_pairs;
}

void Foc_EncoderReset(Foc_Encoder_t *encoder, float raw_mech_angle_rad)
{
  const float aligned = Foc_ApplyEncoderDirToMechTheta(raw_mech_angle_rad, encoder->direction);
  encoder->last_raw_mech = raw_mech_angle_rad;
  encoder->mech_wrapped = aligned;
  encoder->mech_unwrapped = aligned;
  encoder->mech_zero = 0.0f;
  encoder->pll_theta = aligned;
  encoder->vel_hat = 0.0f;
  encoder->elec_angle = Foc_WrapAngle0To2Pi(
      ((float)encoder->pole_pairs * aligned) + encoder->electrical_offset_rad);
  encoder->initialized = 1U;
}

void Foc_EncoderSetZero(Foc_Encoder_t *encoder)
{
  encoder->mech_zero = encoder->mech_unwrapped;
}

static void Foc_EncoderPllCorrect(Foc_Encoder_t *encoder, float meas, float dt_s)
{
  float err;

  encoder->pll_theta += encoder->vel_hat * dt_s;
  err = meas - encoder->pll_theta;
  encoder->vel_hat += encoder->pll_ki * err * dt_s;
  encoder->pll_theta += encoder->pll_kp * err * dt_s;
}

void Foc_EncoderPredict(Foc_Encoder_t *encoder, float dt_s)
{
  if ((encoder->initialized == 0U) || (dt_s <= 0.0f))
  {
    return;
  }
  encoder->pll_theta += encoder->vel_hat * dt_s;
}

uint8_t Foc_EncoderUpdate(Foc_Encoder_t *encoder, float raw_mech_angle_rad, float dt_s)
{
  float aligned;
  float delta;

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
  encoder->last_raw_mech = raw_mech_angle_rad;
  encoder->mech_wrapped = aligned;
  encoder->mech_unwrapped += delta;
  Foc_EncoderPllCorrect(encoder, encoder->mech_unwrapped, dt_s);

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
  return encoder->vel_hat;
}

float Foc_EncoderGetElectrical(const Foc_Encoder_t *encoder)
{
  return encoder->elec_angle;
}

float Foc_EncoderGetMechWrapped(const Foc_Encoder_t *encoder)
{
  return encoder->mech_wrapped;
}

float Foc_EncoderGetLastRaw(const Foc_Encoder_t *encoder)
{
  return encoder->last_raw_mech;
}

uint8_t Foc_EncoderIsReady(const Foc_Encoder_t *encoder)
{
  return encoder->initialized;
}

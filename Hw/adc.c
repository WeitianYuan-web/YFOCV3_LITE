#include "adc.h"

#include "board.h"
#include "config.h"

static volatile uint16_t s_vbus_raw;
static volatile float s_vbus_v;
static uint32_t s_last_poll_ms;

static uint8_t Adc_PollRegularOnce(uint16_t *raw_out)
{
  uint32_t guard = 20000U;

  adc_flag_clear(ADC0, ADC_FLAG_EOC);
  adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
  while ((RESET == adc_flag_get(ADC0, ADC_FLAG_EOC)) && (--guard != 0U))
  {
  }
  if (guard == 0U)
  {
    return 0U;
  }

  *raw_out = adc_routine_data_read(ADC0);
  return 1U;
}

void Adc_Init(void)
{
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_ADC0);
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);

  gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);

  adc_deinit(ADC0);
  adc_mode_config(ADC_MODE_FREE);
  adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
  adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1U);
  adc_routine_channel_config(ADC0, 0U, ADC_CHANNEL_8, ADC_SAMPLETIME_55POINT5);
  adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC0_1_2_EXTTRIG_ROUTINE_NONE);
  adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, ENABLE);

  adc_enable(ADC0);
  {
    volatile uint32_t delay = 10000U;
    while (delay != 0U)
    {
      delay--;
    }
  }
  adc_calibration_enable(ADC0);

  s_vbus_raw = 0U;
  s_vbus_v = 0.0f;
  s_last_poll_ms = 0U;
  Adc_Service();
}

void Adc_Service(void)
{
  uint16_t raw;
  const uint32_t now = HAL_GetTick();

  if ((now - s_last_poll_ms) < CFG_VBUS_POLL_MS)
  {
    return;
  }
  s_last_poll_ms = now;

  if (Adc_PollRegularOnce(&raw) == 0U)
  {
    return;
  }

  s_vbus_raw = raw;
  s_vbus_v = ((float)raw * CFG_ADC_VREF_V * CFG_VBUS_SCALE) / CFG_ADC_FULLSCALE;
}

float Adc_GetVbusVolts(void)
{
  return s_vbus_v;
}

uint16_t Adc_GetVbusRaw(void)
{
  return s_vbus_raw;
}

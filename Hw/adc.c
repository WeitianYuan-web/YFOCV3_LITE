#include "adc.h"

#include "board.h"
#include "config.h"

ADC_HandleTypeDef hadc1;

static volatile uint16_t s_vbus_raw;
static volatile float s_vbus_v;
static uint32_t s_last_poll_ms;

static void Adc_ApplyRegularChannel(void)
{
  ADC_ChannelConfTypeDef cfg = {0};

  cfg.Channel = ADC_CHANNEL_15;
  cfg.Rank = ADC_REGULAR_RANK_1;
  cfg.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  cfg.SingleDiff = ADC_SINGLE_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t Adc_PollRegularOnce(uint16_t *raw_out)
{
  uint32_t guard = 20000U;

  if ((ADC1->CR & ADC_CR_ADEN) == 0U)
  {
    return 0U;
  }
  /* Do not test JADSTART. A later TIM1-triggered current sequence keeps
   * JADSTART set while armed; regular SW start can still run and is
   * preempted for a few microseconds. Stopping or skipping here would
   * freeze Vbus the moment current sampling is added. */
  if ((ADC1->CR & ADC_CR_ADSTART) != 0U)
  {
    return 0U;
  }

  ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
  ADC1->CR |= ADC_CR_ADSTART;
  while (((ADC1->ISR & ADC_ISR_EOC) == 0U) && (--guard != 0U))
  {
  }
  if (guard == 0U)
  {
    ADC1->CR |= ADC_CR_ADSTP;
    return 0U;
  }

  *raw_out = (uint16_t)ADC1->DR;
  return 1U;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adc)
{
  GPIO_InitTypeDef gpio = {0};
  RCC_PeriphCLKInitTypeDef periph = {0};

  if (adc->Instance != ADC1)
  {
    return;
  }

  periph.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  periph.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);
}

void Adc_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  Adc_ApplyRegularChannel();
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Drain the first software conversion so ADSTART is clear; keep ADEN. */
  {
    uint16_t dummy = 0U;
    (void)HAL_ADC_PollForConversion(&hadc1, 2U);
    dummy = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)dummy;
  }

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

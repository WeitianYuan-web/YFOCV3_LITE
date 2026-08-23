#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ADC1 regular = PB0 / IN15 bus voltage, software start.
 * ADC1 injected is left unused for a later 20 kHz TIM1-triggered current
 * sequence (PA0 / IN1). Regular channel is never remapped at runtime, and
 * conversions do not HAL_ADC_Stop / ADDIS, so injected can be added later
 * without fighting a Vbus channel swap.
 */
extern ADC_HandleTypeDef hadc1;

void Adc_Init(void);
void Adc_Service(void);
float Adc_GetVbusVolts(void);
uint16_t Adc_GetVbusRaw(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */

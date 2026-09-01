#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ADC0 regular = PB0 / IN8 bus voltage, software start.
 * Inserted group is left unused for a later 20 kHz TIMER0-triggered
 * current sequence. Regular channel is never remapped at runtime.
 */
void Adc_Init(void);
void Adc_Service(void);
float Adc_GetVbusVolts(void);
uint16_t Adc_GetVbusRaw(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */

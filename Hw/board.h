#ifndef BOARD_H
#define BOARD_H

#include "gd32f30x.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Board_Init(void);
void Board_DwtInit(void);
uint32_t Board_DwtGetCycles(void);
uint32_t Board_DwtCyclesToNs(uint32_t cycles);
void Board_LedSet(uint8_t on);
void Board_LedToggle(void);
uint8_t Board_ButtonRaw(void);
void Error_Handler(void);

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_IncTick(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */

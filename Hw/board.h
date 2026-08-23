#ifndef BOARD_H
#define BOARD_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef   hspi1;
extern TIM_HandleTypeDef   htim1;
extern TIM_HandleTypeDef   htim6;

void Board_Init(void);
void Board_DwtInit(void);
uint32_t Board_DwtGetCycles(void);
uint32_t Board_DwtCyclesToNs(uint32_t cycles);
void Board_LedSet(uint8_t on);
void Board_LedToggle(void);
uint8_t Board_ButtonRaw(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */

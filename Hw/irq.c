#include "board.h"
#include "can.h"
#include "servo.h"

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void TIM1_UP_TIM16_IRQHandler(void)
{
  if ((TIM1->SR & TIM_SR_UIF) != 0U)
  {
    TIM1->SR = ~TIM_SR_UIF;
    Servo_OnPwmIsr();
  }
}

void TIM6_DAC_IRQHandler(void)
{
  if ((TIM6->SR & TIM_SR_UIF) != 0U)
  {
    TIM6->SR = ~TIM_SR_UIF;
    Servo_OnCtrlIsr();
  }
}

void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

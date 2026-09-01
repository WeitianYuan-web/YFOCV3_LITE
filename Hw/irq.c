#include "board.h"
#include "can.h"
#include "servo.h"
#include "gd32f30x.h"

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

void TIMER0_UP_IRQHandler(void)
{
  if (SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP))
  {
    timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
    Servo_OnPwmIsr();
  }
}

void TIMER5_IRQHandler(void)
{
  if (SET == timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP))
  {
    timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
    Servo_OnCtrlIsr();
  }
}

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
  Can_ProcessRxIrq();
}

void CAN0_EWMC_IRQHandler(void)
{
  Can_OnBusOffIrq();
}

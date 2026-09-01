#include "pwm.h"
#include "config.h"
#include "foc_math.h"
#include "gd32f30x.h"

static uint32_t Pwm_DutyToCompare(float duty)
{
  const uint32_t arr = TIMER_CAR(TIMER0);
  duty = Foc_Clamp(duty, 0.0f, 1.0f);
  return (uint32_t)((duty * (float)arr) + 0.5f);
}

void Pwm_StartTimer(void)
{
  const uint32_t mid = (CFG_TIM1_ARR + 1U) / 2U;
  TIMER_CH0CV(TIMER0) = mid;
  TIMER_CH1CV(TIMER0) = mid;
  TIMER_CH2CV(TIMER0) = mid;
  timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
  timer_interrupt_enable(TIMER0, TIMER_INT_UP);
  timer_enable(TIMER0);
}

void Pwm_EnableOutputs(void)
{
  timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
  timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
  timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
  timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
  timer_primary_output_config(TIMER0, ENABLE);
}

void Pwm_DisableOutputs(void)
{
  timer_primary_output_config(TIMER0, DISABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
  timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
  timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
  timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
  timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);
}

void Pwm_ApplyDuty(float duty_a, float duty_b, float duty_c)
{
  TIMER_CH0CV(TIMER0) = Pwm_DutyToCompare(duty_a);
  TIMER_CH1CV(TIMER0) = Pwm_DutyToCompare(duty_b);
  TIMER_CH2CV(TIMER0) = Pwm_DutyToCompare(duty_c);
}

void CtrlTimer_Start(void)
{
  timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
  timer_interrupt_enable(TIMER5, TIMER_INT_UP);
  timer_enable(TIMER5);
}

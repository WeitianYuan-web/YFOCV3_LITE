#include "pwm.h"
#include "board.h"
#include "config.h"
#include "foc_math.h"

static uint32_t Pwm_DutyToCompare(float duty)
{
  const uint32_t arr = TIM1->ARR;
  duty = Foc_Clamp(duty, 0.0f, 1.0f);
  return (uint32_t)((duty * (float)arr) + 0.5f);
}

void Pwm_StartTimer(void)
{
  TIM1->CCR1 = (CFG_TIM1_ARR + 1U) / 2U;
  TIM1->CCR2 = (CFG_TIM1_ARR + 1U) / 2U;
  TIM1->CCR3 = (CFG_TIM1_ARR + 1U) / 2U;
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim1);
}

void Pwm_EnableOutputs(void)
{
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void Pwm_DisableOutputs(void)
{
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void Pwm_ApplyDuty(float duty_a, float duty_b, float duty_c)
{
  TIM1->CCR1 = Pwm_DutyToCompare(duty_a);
  TIM1->CCR2 = Pwm_DutyToCompare(duty_b);
  TIM1->CCR3 = Pwm_DutyToCompare(duty_c);
}

void CtrlTimer_Start(void)
{
  (void)HAL_TIM_Base_Start_IT(&htim6);
}

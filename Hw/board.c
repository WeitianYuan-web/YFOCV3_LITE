#include "board.h"
#include "adc.h"
#include "config.h"

static volatile uint32_t s_tick_ms;

static void Board_GpioInit(void)
{
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_AF);

  gpio_bit_reset(GPIOC, GPIO_PIN_13);
  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

  gpio_bit_set(GPIOA, GPIO_PIN_4);
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

  gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
}

static void Board_SpiInit(void)
{
  spi_parameter_struct spi_init_struct;

  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_SPI0);

  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
  gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
#else
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
#endif

  spi_i2s_deinit(SPI0);
  spi_struct_para_init(&spi_init_struct);
  spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
  spi_init_struct.device_mode = SPI_MASTER;
  spi_init_struct.nss = SPI_NSS_SOFT;
  spi_init_struct.endian = SPI_ENDIAN_MSB;
#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
  spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
  spi_init_struct.prescale = SPI_PSC_16;
#else
  spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
  spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
  spi_init_struct.prescale = SPI_PSC_8;
#endif
  spi_init(SPI0, &spi_init_struct);
  spi_enable(SPI0);
}

static void Board_CanGpioInit(void)
{
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_CAN0);

  /* Default CAN0 is PA11/PA12 (USB D-/D+). Partial remap → PB8/PB9. */
  gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);

  gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
  gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
}

static void Board_Tim0Init(void)
{
  timer_parameter_struct timer_initpara;
  timer_oc_parameter_struct timer_ocinitpara;
  timer_break_parameter_struct timer_breakpara;
  uint16_t pulse_50;

  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_TIMER0);

  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
  gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

  timer_deinit(TIMER0);
  timer_struct_para_init(&timer_initpara);
  timer_initpara.prescaler = 0U;
  timer_initpara.alignedmode = TIMER_COUNTER_CENTER_UP;
  timer_initpara.counterdirection = TIMER_COUNTER_UP;
  timer_initpara.period = (uint16_t)CFG_TIM1_ARR;
  timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
  timer_initpara.repetitioncounter = 1U;
  timer_init(TIMER0, &timer_initpara);

  timer_channel_output_struct_para_init(&timer_ocinitpara);
  timer_ocinitpara.outputstate = TIMER_CCX_DISABLE;
  timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
  timer_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
  timer_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
  timer_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
  timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

  pulse_50 = (uint16_t)((CFG_TIM1_ARR + 1U) / 2U);
  timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocinitpara);
  timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocinitpara);
  timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocinitpara);
  timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pulse_50);
  timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pulse_50);
  timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, pulse_50);
  timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
  timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
  timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
  timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
  timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
  timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);

  timer_break_struct_para_init(&timer_breakpara);
  timer_breakpara.runoffstate = TIMER_ROS_STATE_ENABLE;
  timer_breakpara.ideloffstate = TIMER_IOS_STATE_ENABLE;
  timer_breakpara.deadtime = (uint16_t)CFG_TIM1_DEADTIME_DTG;
  timer_breakpara.breakpolarity = TIMER_BREAK_POLARITY_HIGH;
  timer_breakpara.outputautostate = TIMER_OUTAUTO_DISABLE;
  timer_breakpara.protectmode = TIMER_CCHP_PROT_OFF;
  timer_breakpara.breakstate = TIMER_BREAK_DISABLE;
  timer_break_config(TIMER0, &timer_breakpara);

  timer_auto_reload_shadow_enable(TIMER0);
  timer_primary_output_config(TIMER0, DISABLE);

  nvic_irq_enable(TIMER0_UP_IRQn, CFG_NVIC_TIM1, 0U);
}

static void Board_Tim5Init(void)
{
  timer_parameter_struct timer_initpara;

  rcu_periph_clock_enable(RCU_TIMER5);
  timer_deinit(TIMER5);
  timer_struct_para_init(&timer_initpara);
  timer_initpara.prescaler = (uint16_t)CFG_CTRL_TIM_PSC;
  timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
  timer_initpara.counterdirection = TIMER_COUNTER_UP;
  timer_initpara.period = (uint16_t)CFG_CTRL_TIM_ARR;
  timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
  timer_initpara.repetitioncounter = 0U;
  timer_init(TIMER5, &timer_initpara);

  nvic_irq_enable(TIMER5_IRQn, CFG_NVIC_TIM6, 0U);
}

static void Board_SystickInit(void)
{
  if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
  {
    Error_Handler();
  }
  NVIC_SetPriority(SysTick_IRQn, 15U);
}

void Board_Init(void)
{
  nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
  Board_SystickInit();
  Board_GpioInit();
  Board_CanGpioInit();
  Board_SpiInit();
  Board_Tim0Init();
  Board_Tim5Init();
  Board_DwtInit();
  Adc_Init();
}

void Board_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Board_DwtGetCycles(void)
{
  return DWT->CYCCNT;
}

uint32_t Board_DwtCyclesToNs(uint32_t cycles)
{
  return (uint32_t)(((uint64_t)cycles * 1000000000ULL) / (uint64_t)CFG_SYSCLK_HZ);
}

void Board_LedSet(uint8_t on)
{
  gpio_bit_write(GPIOC, GPIO_PIN_13, (on != 0U) ? SET : RESET);
}

void Board_LedToggle(void)
{
  gpio_bit_write(GPIOC, GPIO_PIN_13,
                 (RESET == gpio_output_bit_get(GPIOC, GPIO_PIN_13)) ? SET : RESET);
}

uint8_t Board_ButtonRaw(void)
{
  return (RESET == gpio_input_bit_get(GPIOB, GPIO_PIN_3)) ? 1U : 0U;
}

void HAL_IncTick(void)
{
  s_tick_ms++;
}

uint32_t HAL_GetTick(void)
{
  return s_tick_ms;
}

void HAL_Delay(uint32_t ms)
{
  const uint32_t start = s_tick_ms;
  while ((s_tick_ms - start) < ms)
  {
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

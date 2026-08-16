#include "encoder.h"
#include "board.h"
#include "stm32g4xx.h"

#define ENCODER_SPI_FAST_GUARD  (4000U)
#define ENCODER_CS_SETUP_NOPS   (40U)

#if (CFG_ENCODER_TYPE == CFG_ENCODER_MT6701)
static uint8_t Mt6701_CalcCrc6(uint32_t data18)
{
  uint8_t crc = 0U;
  int8_t bit;

  for (bit = 17; bit >= 0; bit--)
  {
    uint8_t in = (uint8_t)((data18 >> (uint8_t)bit) & 0x1U);
    uint8_t msb = (uint8_t)((crc >> 5U) & 0x1U);
    crc = (uint8_t)((crc << 1U) & 0x3FU);
    if ((msb ^ in) != 0U)
    {
      crc ^= 0x03U;
    }
  }
  return (uint8_t)(crc & 0x3FU);
}

static uint16_t Mt6701_Parse(const uint8_t raw[3], uint8_t *status_out, uint8_t *crc_ok_out)
{
  uint32_t frame24;
  uint16_t angle;
  uint8_t crc_rx;
  uint32_t data18;

  if ((raw[0] == 0U) && (raw[1] == 0U) && (raw[2] == 0U))
  {
    if (crc_ok_out != 0)
    {
      *crc_ok_out = 0U;
    }
    if (status_out != 0)
    {
      *status_out = 0U;
    }
    return ENCODER_READ_INVALID;
  }

  frame24 = ((uint32_t)raw[0] << 16U) | ((uint32_t)raw[1] << 8U) | (uint32_t)raw[2];
  angle = (uint16_t)((frame24 >> 10U) & 0x3FFFU);
  if (status_out != 0)
  {
    *status_out = (uint8_t)((frame24 >> 6U) & 0x0FU);
  }
  crc_rx = (uint8_t)(frame24 & 0x3FU);
  data18 = (frame24 >> 6U) & 0x3FFFFU;

  if (crc_ok_out != 0)
  {
    *crc_ok_out = (Mt6701_CalcCrc6(data18) == crc_rx) ? 1U : 0U;
  }
  return angle;
}

static void Encoder_FlushRx8(void)
{
  SPI_TypeDef *spi = SPI1;
  volatile uint8_t *spi_dr8 = (volatile uint8_t *)&spi->DR;
  uint32_t guard = 16U;

  while (((spi->SR & SPI_SR_RXNE) != 0U) && (--guard != 0U))
  {
    (void)*spi_dr8;
  }
}

static uint8_t Encoder_PumpBytes8(uint8_t *rx, uint16_t length)
{
  SPI_TypeDef *spi = SPI1;
  volatile uint8_t *spi_dr8 = (volatile uint8_t *)&spi->DR;
  uint16_t tx_index = 0U;
  uint16_t rx_index = 0U;
  uint32_t guard = ENCODER_SPI_FAST_GUARD;

  spi->CR1 |= SPI_CR1_SPE;
  Encoder_FlushRx8();

  while (rx_index < length)
  {
    if ((tx_index < length) && ((spi->SR & SPI_SR_TXE) != 0U))
    {
      *spi_dr8 = 0U;
      tx_index++;
    }

    if ((spi->SR & SPI_SR_RXNE) != 0U)
    {
      rx[rx_index] = *spi_dr8;
      rx_index++;
      guard = ENCODER_SPI_FAST_GUARD;
    }
    else if (--guard == 0U)
    {
      break;
    }
  }

  if (rx_index != length)
  {
    return 0U;
  }

  guard = ENCODER_SPI_FAST_GUARD;
  while (((spi->SR & SPI_SR_BSY) != 0U) && (--guard != 0U))
  {
  }
  return (guard != 0U) ? 1U : 0U;
}
#endif

#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
static uint8_t Encoder_PumpWord16(uint16_t *rx)
{
  SPI_TypeDef *spi = SPI1;
  volatile uint16_t *spi_dr16 = (volatile uint16_t *)&spi->DR;
  uint32_t guard = ENCODER_SPI_FAST_GUARD;

  spi->CR1 |= SPI_CR1_SPE;

  while (((spi->SR & SPI_SR_TXE) == 0U) && (--guard != 0U))
  {
  }
  if (guard == 0U)
  {
    return 0U;
  }
  *spi_dr16 = 0U;

  guard = ENCODER_SPI_FAST_GUARD;
  while (((spi->SR & SPI_SR_RXNE) == 0U) && (--guard != 0U))
  {
  }
  if (guard == 0U)
  {
    return 0U;
  }
  *rx = *spi_dr16;

  guard = ENCODER_SPI_FAST_GUARD;
  while (((spi->SR & SPI_SR_BSY) != 0U) && (--guard != 0U))
  {
  }
  return (guard != 0U) ? 1U : 0U;
}
#endif

static void Encoder_CsSetupDelay(void)
{
  uint32_t i;
  for (i = 0U; i < ENCODER_CS_SETUP_NOPS; i++)
  {
    __NOP();
  }
}

void Encoder_Init(void)
{
  GPIOA->BSRR = GPIO_PIN_4;
  SPI1->CR1 |= SPI_CR1_SPE;
#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
  {
    uint16_t dummy = 0U;
    GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
    Encoder_CsSetupDelay();
    (void)Encoder_PumpWord16(&dummy);
    GPIOA->BSRR = GPIO_PIN_4;
  }
#endif
}

#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
uint8_t Encoder_ReadFrame(uint16_t *raw_out, uint8_t *status_out)
{
  uint16_t rx = 0U;
  uint8_t attempt;
  uint32_t primask;

  if (raw_out == 0)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  /* Same as YFOCV3_ST: two CS frames of 0x0000, angle is in the second. */
  for (attempt = 0U; attempt < 2U; attempt++)
  {
    GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
    Encoder_CsSetupDelay();
    if (Encoder_PumpWord16(&rx) == 0U)
    {
      GPIOA->BSRR = GPIO_PIN_4;
      continue;
    }
    GPIOA->BSRR = GPIO_PIN_4;
    Encoder_CsSetupDelay();

    GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
    Encoder_CsSetupDelay();
    if (Encoder_PumpWord16(&rx) == 0U)
    {
      GPIOA->BSRR = GPIO_PIN_4;
      continue;
    }
    GPIOA->BSRR = GPIO_PIN_4;

    *raw_out = rx;
    if (status_out != 0)
    {
      *status_out = 0U;
    }
    __set_PRIMASK(primask);
    return 1U;
  }

  __set_PRIMASK(primask);
  return 0U;
}
#else
uint8_t Encoder_ReadFrame(uint16_t *raw_out, uint8_t *status_out)
{
  uint8_t rx[3] = {0U, 0U, 0U};
  uint16_t angle;
  uint8_t crc_ok = 0U;
  uint8_t status = 0U;
  uint8_t attempt;
  uint32_t primask;

  if (raw_out == 0)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  for (attempt = 0U; attempt < 2U; attempt++)
  {
    GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
    Encoder_CsSetupDelay();
    if (Encoder_PumpBytes8(rx, 3U) == 0U)
    {
      GPIOA->BSRR = GPIO_PIN_4;
      continue;
    }
    GPIOA->BSRR = GPIO_PIN_4;

    angle = Mt6701_Parse(rx, &status, &crc_ok);
    if ((angle != ENCODER_READ_INVALID) && (crc_ok != 0U))
    {
      *raw_out = angle;
      if (status_out != 0)
      {
        *status_out = status;
      }
      __set_PRIMASK(primask);
      return 1U;
    }
  }

  __set_PRIMASK(primask);
  return 0U;
}
#endif

uint8_t Encoder_ReadRawFast(uint16_t *raw_out)
{
  return Encoder_ReadFrame(raw_out, 0);
}

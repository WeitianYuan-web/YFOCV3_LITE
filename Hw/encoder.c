#include "encoder.h"
#include "board.h"
#include "stm32g4xx.h"

#define ENCODER_SPI_FAST_GUARD  (2000U)

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

static uint16_t Mt6701_Parse(const uint8_t raw[3], uint8_t *crc_ok_out)
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
    return ENCODER_READ_INVALID;
  }

  frame24 = ((uint32_t)raw[0] << 16U) | ((uint32_t)raw[1] << 8U) | (uint32_t)raw[2];
  angle = (uint16_t)((frame24 >> 10U) & 0x3FFFU);
  crc_rx = (uint8_t)(frame24 & 0x3FU);
  data18 = (frame24 >> 6U) & 0x3FFFFU;

  if (crc_ok_out != 0)
  {
    *crc_ok_out = (Mt6701_CalcCrc6(data18) == crc_rx) ? 1U : 0U;
  }
  return angle;
}

static uint8_t Encoder_PumpBytes8(uint8_t *rx, uint16_t length)
{
  SPI_TypeDef *spi = SPI1;
  volatile uint8_t *spi_dr8 = (volatile uint8_t *)&spi->DR;
  uint16_t tx_index = 0U;
  uint16_t rx_index = 0U;
  uint32_t guard = ENCODER_SPI_FAST_GUARD;

  spi->CR1 |= SPI_CR1_SPE;

  while (rx_index < length)
  {
    if ((tx_index < length) && ((spi->SR & SPI_SR_TXE) != 0U))
    {
      *spi_dr8 = 0U;
      tx_index++;
    }

    if ((spi->SR & SPI_SR_RXNE) != 0U)
    {
      uint8_t byte = *spi_dr8;
      rx[rx_index] = byte;
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

void Encoder_Init(void)
{
  GPIOA->BSRR = GPIO_PIN_4;
}

uint8_t Encoder_ReadRawFast(uint16_t *raw_out)
{
  uint8_t rx[3] = {0U, 0U, 0U};
  uint16_t angle;
  uint8_t crc_ok = 0U;
  uint8_t attempt;

  if (raw_out == 0)
  {
    return 0U;
  }

  for (attempt = 0U; attempt < 2U; attempt++)
  {
    GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
    if (Encoder_PumpBytes8(rx, 3U) == 0U)
    {
      GPIOA->BSRR = GPIO_PIN_4;
      continue;
    }
    GPIOA->BSRR = GPIO_PIN_4;

    angle = Mt6701_Parse(rx, &crc_ok);
    if ((angle != ENCODER_READ_INVALID) && (crc_ok != 0U))
    {
      *raw_out = angle;
      return 1U;
    }
  }

  return 0U;
}

#include "encoder.h"
#include "gd32f30x.h"

#define ENCODER_SPI           SPI0
#define ENCODER_CS_PIN        GPIO_PIN_4
#define ENCODER_SPI_FAST_GUARD  (4000U)
#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
#define ENCODER_CS_SETUP_NOPS   (16U)
#else
#define ENCODER_CS_SETUP_NOPS   (12U)
#endif

static void Encoder_CsHigh(void)
{
  GPIO_BOP(GPIOA) = ENCODER_CS_PIN;
}

static void Encoder_CsLow(void)
{
  GPIO_BC(GPIOA) = ENCODER_CS_PIN;
}

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
  volatile uint8_t *spi_dr8 = (volatile uint8_t *)(uint32_t)(ENCODER_SPI + 0x0CU);
  uint32_t guard = 16U;

  while ((0U != (SPI_STAT(ENCODER_SPI) & SPI_STAT_RBNE)) && (--guard != 0U))
  {
    (void)*spi_dr8;
  }
  if (0U != (SPI_STAT(ENCODER_SPI) & SPI_STAT_RXORERR))
  {
    (void)*spi_dr8;
    (void)SPI_STAT(ENCODER_SPI);
  }
}

static uint8_t Encoder_PumpBytes8(uint8_t *rx, uint16_t length)
{
  volatile uint8_t *spi_dr8 = (volatile uint8_t *)(uint32_t)(ENCODER_SPI + 0x0CU);
  uint16_t tx_index = 0U;
  uint16_t rx_index = 0U;
  uint32_t guard = ENCODER_SPI_FAST_GUARD;

  SPI_CTL0(ENCODER_SPI) |= SPI_CTL0_SPIEN;
  Encoder_FlushRx8();

  while ((tx_index < length) && (0U != (SPI_STAT(ENCODER_SPI) & SPI_STAT_TBE)))
  {
    *spi_dr8 = 0U;
    tx_index++;
  }

  while (rx_index < length)
  {
    if ((tx_index < length) && (0U != (SPI_STAT(ENCODER_SPI) & SPI_STAT_TBE)))
    {
      *spi_dr8 = 0U;
      tx_index++;
    }

    if (0U != (SPI_STAT(ENCODER_SPI) & SPI_STAT_RBNE))
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

  return (rx_index == length) ? 1U : 0U;
}
#endif

#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
static uint8_t Encoder_PumpWord16(uint16_t *rx)
{
  volatile uint16_t *spi_dr16 = (volatile uint16_t *)(uint32_t)(ENCODER_SPI + 0x0CU);
  uint32_t guard = ENCODER_SPI_FAST_GUARD;

  SPI_CTL0(ENCODER_SPI) |= SPI_CTL0_SPIEN;

  while ((0U == (SPI_STAT(ENCODER_SPI) & SPI_STAT_TBE)) && (--guard != 0U))
  {
  }
  if (guard == 0U)
  {
    return 0U;
  }
  *spi_dr16 = 0U;

  guard = ENCODER_SPI_FAST_GUARD;
  while ((0U == (SPI_STAT(ENCODER_SPI) & SPI_STAT_RBNE)) && (--guard != 0U))
  {
  }
  if (guard == 0U)
  {
    return 0U;
  }
  *rx = *spi_dr16;

  guard = ENCODER_SPI_FAST_GUARD;
  while ((0U != (SPI_STAT(ENCODER_SPI) & SPI_STAT_TRANS)) && (--guard != 0U))
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
  Encoder_CsHigh();
  SPI_CTL0(ENCODER_SPI) |= SPI_CTL0_SPIEN;
#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
  {
    uint16_t dummy = 0U;
    Encoder_CsLow();
    Encoder_CsSetupDelay();
    (void)Encoder_PumpWord16(&dummy);
    Encoder_CsHigh();
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

  for (attempt = 0U; attempt < 2U; attempt++)
  {
    Encoder_CsLow();
    Encoder_CsSetupDelay();
    if (Encoder_PumpWord16(&rx) == 0U)
    {
      Encoder_CsHigh();
      continue;
    }
    Encoder_CsHigh();
    Encoder_CsSetupDelay();

    Encoder_CsLow();
    Encoder_CsSetupDelay();
    if (Encoder_PumpWord16(&rx) == 0U)
    {
      Encoder_CsHigh();
      continue;
    }
    Encoder_CsHigh();

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
    Encoder_CsLow();
    Encoder_CsSetupDelay();
    if (Encoder_PumpBytes8(rx, 3U) == 0U)
    {
      Encoder_CsHigh();
      continue;
    }
    Encoder_CsHigh();

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

#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
uint8_t Encoder_ReadRawFast(uint16_t *raw_out)
{
  uint16_t rx = 0U;

  if (raw_out == 0)
  {
    return 0U;
  }

  Encoder_CsLow();
  Encoder_CsSetupDelay();
  if (Encoder_PumpWord16(&rx) == 0U)
  {
    Encoder_CsHigh();
    return 0U;
  }
  Encoder_CsHigh();
  Encoder_CsSetupDelay();

  Encoder_CsLow();
  Encoder_CsSetupDelay();
  if (Encoder_PumpWord16(&rx) == 0U)
  {
    Encoder_CsHigh();
    return 0U;
  }
  Encoder_CsHigh();
  *raw_out = rx;
  return 1U;
}
#else
uint8_t Encoder_ReadRawFast(uint16_t *raw_out)
{
  uint8_t rx[3] = {0U, 0U, 0U};
  uint32_t frame24;

  if (raw_out == 0)
  {
    return 0U;
  }

  Encoder_CsLow();
  Encoder_CsSetupDelay();
  if (Encoder_PumpBytes8(rx, 3U) == 0U)
  {
    Encoder_CsHigh();
    return 0U;
  }
  Encoder_CsHigh();

  frame24 = ((uint32_t)rx[0] << 16U) | ((uint32_t)rx[1] << 8U) | (uint32_t)rx[2];
  if (frame24 == 0U)
  {
    return 0U;
  }
  *raw_out = (uint16_t)((frame24 >> 10U) & 0x3FFFU);
  return 1U;
}
#endif

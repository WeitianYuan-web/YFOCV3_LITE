#include "cali_nv.h"

#include <stddef.h>
#include <string.h>

#include "config.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"

#define CALI_NV_MAGIC     (0x59464F43UL) /* "YFOC" */
#define CALI_NV_VERSION   (1U)
#define CALI_NV_PAGE      (63U)
#define CALI_NV_ADDR      (0x0801F800UL)

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t pole_pairs;
  int16_t  encoder_dir;
  int16_t  closed_loop_dir;
  float    electrical_offset_rad;
  uint32_t reserved[3];
  uint32_t crc32;
} CaliNvRecord_t;

_Static_assert(sizeof(CaliNvRecord_t) == 32U, "CaliNvRecord_t must be 32 bytes");

static uint32_t CaliNv_Crc32(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;
  uint32_t j;

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (j = 0U; j < 8U; j++)
    {
      const uint32_t mask = 0U - (crc & 1UL);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static uint32_t CaliNv_RecordCrc(const CaliNvRecord_t *rec)
{
  return CaliNv_Crc32((const uint8_t *)rec, offsetof(CaliNvRecord_t, crc32));
}

static uint8_t CaliNv_RecordValid(const CaliNvRecord_t *rec)
{
  if (rec->magic != CALI_NV_MAGIC)
  {
    return 0U;
  }
  if (rec->version != CALI_NV_VERSION)
  {
    return 0U;
  }
  if ((rec->pole_pairs < (uint16_t)CFG_CALI_PP_MIN) ||
      (rec->pole_pairs > (uint16_t)CFG_CALI_PP_MAX))
  {
    return 0U;
  }
  if ((rec->encoder_dir != 1) && (rec->encoder_dir != -1))
  {
    return 0U;
  }
  if ((rec->closed_loop_dir != 1) && (rec->closed_loop_dir != -1))
  {
    return 0U;
  }
  if (rec->crc32 != CaliNv_RecordCrc(rec))
  {
    return 0U;
  }
  return 1U;
}

static uint32_t CaliNv_IrqQuiesce(void)
{
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  __DSB();
  __ISB();
  return primask;
}

static void CaliNv_IrqResume(uint32_t primask)
{
  TIM1->SR = (uint32_t)~TIM_SR_UIF;
  TIM6->SR = (uint32_t)~TIM_SR_UIF;
  NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
  NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  NVIC_ClearPendingIRQ(FDCAN1_IT0_IRQn);
  NVIC_ClearPendingIRQ(SysTick_IRQn);
  __DSB();
  __ISB();
  __set_PRIMASK(primask);
}

static uint8_t CaliNv_EraseUnlocked(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0xFFFFFFFFUL;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = CALI_NV_PAGE;
  erase.NbPages = 1U;
  return (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK) ? 1U : 0U;
}

uint8_t CaliNv_Load(CaliNvData_t *out)
{
  CaliNvRecord_t rec;

  if (out == 0)
  {
    return 0U;
  }

  __DSB();
  memcpy(&rec, (const void *)CALI_NV_ADDR, sizeof(rec));
  __DSB();

  if (CaliNv_RecordValid(&rec) == 0U)
  {
    return 0U;
  }

  out->pole_pairs = (uint8_t)rec.pole_pairs;
  out->encoder_dir = (int8_t)rec.encoder_dir;
  out->closed_loop_dir = (int8_t)rec.closed_loop_dir;
  out->electrical_offset_rad = rec.electrical_offset_rad;
  return 1U;
}

uint8_t CaliNv_Save(const CaliNvData_t *in)
{
  CaliNvRecord_t rec;
  uint64_t words[4];
  uint32_t i;
  uint8_t ok = 0U;

  if (in == 0)
  {
    return 0U;
  }
  if ((in->pole_pairs < CFG_CALI_PP_MIN) || (in->pole_pairs > CFG_CALI_PP_MAX))
  {
    return 0U;
  }

  memset(&rec, 0, sizeof(rec));
  rec.magic = CALI_NV_MAGIC;
  rec.version = CALI_NV_VERSION;
  rec.pole_pairs = (uint16_t)in->pole_pairs;
  rec.encoder_dir = (int16_t)((in->encoder_dir < 0) ? -1 : 1);
  rec.closed_loop_dir = (int16_t)((in->closed_loop_dir < 0) ? -1 : 1);
  rec.electrical_offset_rad = in->electrical_offset_rad;
  rec.crc32 = CaliNv_RecordCrc(&rec);
  memcpy(words, &rec, sizeof(rec));

  {
    const uint32_t primask = CaliNv_IrqQuiesce();

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
      CaliNv_IrqResume(primask);
      return 0U;
    }
    if (CaliNv_EraseUnlocked() != 0U)
    {
      ok = 1U;
      for (i = 0U; i < 4U; i++)
      {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              CALI_NV_ADDR + (i * 8UL),
                              words[i]) != HAL_OK)
        {
          ok = 0U;
          break;
        }
      }
    }
    (void)HAL_FLASH_Lock();
    __DSB();
    CaliNv_IrqResume(primask);
  }

  if (ok != 0U)
  {
    CaliNvData_t check;
    if ((CaliNv_Load(&check) == 0U) ||
        (check.pole_pairs != rec.pole_pairs) ||
        (check.encoder_dir != rec.encoder_dir) ||
        (check.closed_loop_dir != rec.closed_loop_dir))
    {
      ok = 0U;
    }
  }
  return ok;
}

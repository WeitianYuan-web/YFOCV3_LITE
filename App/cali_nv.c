#include "cali_nv.h"

#include <stddef.h>
#include <string.h>

#include "config.h"
#include "gd32f30x.h"

#define CALI_NV_MAGIC     (0x59464F43UL) /* "YFOC" */
#define CALI_NV_VERSION_V1 (1U)
#define CALI_NV_VERSION   (2U)
#define CALI_NV_ADDR      (0x0803F800UL) /* last 2 KB page of 256 KB Flash */
#define CALI_NV_FLAG_GAINS (1U)

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

static uint8_t CaliNv_HeaderOk(const CaliNvRecord_t *rec)
{
  if (rec->magic != CALI_NV_MAGIC)
  {
    return 0U;
  }
  if ((rec->version != CALI_NV_VERSION_V1) && (rec->version != CALI_NV_VERSION))
  {
    return 0U;
  }
  if (rec->crc32 != CaliNv_RecordCrc(rec))
  {
    return 0U;
  }
  return 1U;
}

static uint8_t CaliNv_ReadRaw(CaliNvRecord_t *rec)
{
  if (rec == 0)
  {
    return 0U;
  }
  __DSB();
  memcpy(rec, (const void *)CALI_NV_ADDR, sizeof(*rec));
  __DSB();
  return CaliNv_HeaderOk(rec);
}

static uint8_t CaliNv_NodeIdOk(uint32_t id)
{
  const uint8_t nid = (uint8_t)(id & 0xFFU);
  return ((nid >= CFG_NODE_ID_MIN) && (nid <= CFG_NODE_ID_MAX)) ? 1U : 0U;
}

static void CaliNv_ClearGains(CaliNvData_t *out)
{
  out->user_gains_valid = 0U;
  out->vel_kp_raw = 0U;
  out->vel_ki_raw = 0U;
  out->pos_kp_raw = 0U;
  out->pos_ki_raw = 0U;
  out->pos_kd_raw = 0U;
}

static void CaliNv_Unpack(const CaliNvRecord_t *rec, CaliNvData_t *out)
{
  const uint32_t w0 = rec->reserved[0];
  const uint32_t w1 = rec->reserved[1];
  const uint32_t w2 = rec->reserved[2];

  out->pole_pairs = (uint8_t)rec->pole_pairs;
  out->encoder_dir = (int8_t)rec->encoder_dir;
  out->closed_loop_dir = (int8_t)rec->closed_loop_dir;
  out->electrical_offset_rad = rec->electrical_offset_rad;
  out->node_id = CaliNv_NodeIdOk(w0) ? (uint8_t)(w0 & 0xFFU) : 0U;
  if (rec->version >= CALI_NV_VERSION)
  {
    out->user_gains_valid = (((w0 >> 8) & 0xFFU) & CALI_NV_FLAG_GAINS) ? 1U : 0U;
    out->vel_kp_raw = (uint16_t)(w1 & 0xFFFFU);
    out->vel_ki_raw = (uint16_t)(w1 >> 16);
    out->pos_kp_raw = (uint16_t)(w2 & 0xFFFFU);
    out->pos_ki_raw = (uint16_t)(w2 >> 16);
    out->pos_kd_raw = (uint16_t)(w0 >> 16);
  }
  else
  {
    CaliNv_ClearGains(out);
  }
}

static void CaliNv_Pack(CaliNvRecord_t *rec, const CaliNvData_t *in)
{
  uint8_t flags = 0U;
  uint16_t pos_kd = 0U;
  uint16_t vel_kp = 0U;
  uint16_t vel_ki = 0U;
  uint16_t pos_kp = 0U;
  uint16_t pos_ki = 0U;

  memset(rec, 0, sizeof(*rec));
  rec->magic = CALI_NV_MAGIC;
  rec->version = CALI_NV_VERSION;
  rec->pole_pairs = (uint16_t)in->pole_pairs;
  rec->encoder_dir = (int16_t)((in->encoder_dir < 0) ? -1 : 1);
  rec->closed_loop_dir = (int16_t)((in->closed_loop_dir < 0) ? -1 : 1);
  rec->electrical_offset_rad = in->electrical_offset_rad;
  if (in->user_gains_valid != 0U)
  {
    flags = CALI_NV_FLAG_GAINS;
    vel_kp = in->vel_kp_raw;
    vel_ki = in->vel_ki_raw;
    pos_kp = in->pos_kp_raw;
    pos_ki = in->pos_ki_raw;
    pos_kd = in->pos_kd_raw;
  }
  rec->reserved[0] = (uint32_t)(CaliNv_NodeIdOk(in->node_id) ? in->node_id : 0U) |
                     ((uint32_t)flags << 8) |
                     ((uint32_t)pos_kd << 16);
  rec->reserved[1] = (uint32_t)vel_kp | ((uint32_t)vel_ki << 16);
  rec->reserved[2] = (uint32_t)pos_kp | ((uint32_t)pos_ki << 16);
  rec->crc32 = CaliNv_RecordCrc(rec);
}

static uint8_t CaliNv_ReadData(CaliNvData_t *out)
{
  CaliNvRecord_t rec;

  if ((out == 0) || (CaliNv_ReadRaw(&rec) == 0U))
  {
    return 0U;
  }
  CaliNv_Unpack(&rec, out);
  return 1U;
}

static uint8_t CaliNv_Program(const CaliNvRecord_t *rec);

static uint8_t CaliNv_WriteData(const CaliNvData_t *in)
{
  CaliNvRecord_t rec;

  CaliNv_Pack(&rec, in);
  return CaliNv_Program(&rec);
}

static uint8_t CaliNv_RecordValid(const CaliNvRecord_t *rec)
{
  if (CaliNv_HeaderOk(rec) == 0U)
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
  timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
  timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
  NVIC_ClearPendingIRQ(TIMER0_UP_IRQn);
  NVIC_ClearPendingIRQ(TIMER5_IRQn);
  NVIC_ClearPendingIRQ(USBD_LP_CAN0_RX0_IRQn);
  NVIC_ClearPendingIRQ(CAN0_EWMC_IRQn);
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
  __DSB();
  __ISB();
  __set_PRIMASK(primask);
}

static uint8_t CaliNv_Program(const CaliNvRecord_t *rec)
{
  uint32_t words[8];
  uint32_t i;
  uint8_t ok = 0U;
  const uint32_t primask = CaliNv_IrqQuiesce();

  memcpy(words, rec, sizeof(*rec));
  fmc_unlock();
  if (FMC_READY == fmc_page_erase(CALI_NV_ADDR))
  {
    ok = 1U;
    for (i = 0U; i < 8U; i++)
    {
      if (FMC_READY != fmc_word_program(CALI_NV_ADDR + (i * 4UL), words[i]))
      {
        ok = 0U;
        break;
      }
    }
  }
  fmc_lock();
  __DSB();
  CaliNv_IrqResume(primask);
  return ok;
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

  CaliNv_Unpack(&rec, out);
  return 1U;
}

uint8_t CaliNv_LoadNodeId(void)
{
  CaliNvData_t data;

  if (CaliNv_ReadData(&data) == 0U)
  {
    return 0U;
  }
  return data.node_id;
}

uint8_t CaliNv_Save(const CaliNvData_t *in)
{
  CaliNvData_t rec;
  uint8_t ok;

  if (in == 0)
  {
    return 0U;
  }
  if ((in->pole_pairs < CFG_CALI_PP_MIN) || (in->pole_pairs > CFG_CALI_PP_MAX))
  {
    return 0U;
  }

  if (CaliNv_ReadData(&rec) == 0U)
  {
    memset(&rec, 0, sizeof(rec));
  }
  rec.pole_pairs = in->pole_pairs;
  rec.encoder_dir = (in->encoder_dir < 0) ? -1 : 1;
  rec.closed_loop_dir = (in->closed_loop_dir < 0) ? -1 : 1;
  rec.electrical_offset_rad = in->electrical_offset_rad;
  rec.node_id = CaliNv_NodeIdOk(in->node_id) ? in->node_id : rec.node_id;

  ok = CaliNv_WriteData(&rec);
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

uint8_t CaliNv_SaveNodeId(uint8_t node_id)
{
  CaliNvData_t rec;

  if (CaliNv_NodeIdOk(node_id) == 0U)
  {
    return 0U;
  }

  if (CaliNv_ReadData(&rec) == 0U)
  {
    memset(&rec, 0, sizeof(rec));
  }
  rec.node_id = node_id;
  if (CaliNv_WriteData(&rec) == 0U)
  {
    return 0U;
  }
  return (CaliNv_LoadNodeId() == node_id) ? 1U : 0U;
}

uint8_t CaliNv_SaveUserGains(uint8_t node_id,
                             uint16_t vel_kp_raw,
                             uint16_t vel_ki_raw,
                             uint16_t pos_kp_raw,
                             uint16_t pos_ki_raw,
                             uint16_t pos_kd_raw)
{
  CaliNvData_t rec;

  if (CaliNv_ReadData(&rec) == 0U)
  {
    memset(&rec, 0, sizeof(rec));
    rec.node_id = CaliNv_NodeIdOk(node_id) ? node_id : 0U;
  }
  rec.user_gains_valid = 1U;
  rec.vel_kp_raw = vel_kp_raw;
  rec.vel_ki_raw = vel_ki_raw;
  rec.pos_kp_raw = pos_kp_raw;
  rec.pos_ki_raw = pos_ki_raw;
  rec.pos_kd_raw = pos_kd_raw;
  if (CaliNv_WriteData(&rec) == 0U)
  {
    return 0U;
  }
  if (CaliNv_ReadData(&rec) == 0U)
  {
    return 0U;
  }
  return ((rec.user_gains_valid != 0U) &&
          (rec.vel_kp_raw == vel_kp_raw) &&
          (rec.vel_ki_raw == vel_ki_raw) &&
          (rec.pos_kp_raw == pos_kp_raw) &&
          (rec.pos_ki_raw == pos_ki_raw) &&
          (rec.pos_kd_raw == pos_kd_raw)) ? 1U : 0U;
}

#ifndef CALI_NV_H
#define CALI_NV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t pole_pairs;
  int8_t  encoder_dir;
  int8_t  closed_loop_dir;
  uint8_t node_id;
  uint8_t user_gains_valid;
  float   electrical_offset_rad;
  uint16_t vel_kp_raw;
  uint16_t vel_ki_raw;
  uint16_t pos_kp_raw;
  uint16_t pos_ki_raw;
  uint16_t pos_kd_raw;
} CaliNvData_t;

uint8_t CaliNv_Load(CaliNvData_t *out);
uint8_t CaliNv_Save(const CaliNvData_t *in);
uint8_t CaliNv_LoadNodeId(void);
uint8_t CaliNv_SaveNodeId(uint8_t node_id);
uint8_t CaliNv_SaveUserGains(uint8_t node_id,
                             uint16_t vel_kp_raw,
                             uint16_t vel_ki_raw,
                             uint16_t pos_kp_raw,
                             uint16_t pos_ki_raw,
                             uint16_t pos_kd_raw);

#ifdef __cplusplus
}
#endif

#endif /* CALI_NV_H */

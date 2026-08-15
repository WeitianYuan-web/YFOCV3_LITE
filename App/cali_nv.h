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
  float   electrical_offset_rad;
} CaliNvData_t;

uint8_t CaliNv_Load(CaliNvData_t *out);
uint8_t CaliNv_Save(const CaliNvData_t *in);

#ifdef __cplusplus
}
#endif

#endif /* CALI_NV_H */

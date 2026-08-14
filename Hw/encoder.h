#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_CPR           16384U
#define ENCODER_READ_INVALID  0xFFFFU

void Encoder_Init(void);
uint8_t Encoder_ReadRawFast(uint16_t *raw_out);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */

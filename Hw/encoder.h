#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (CFG_ENCODER_TYPE == CFG_ENCODER_KTH7812)
#define ENCODER_CPR           65536U
#else
#define ENCODER_CPR           16384U
#endif
#define ENCODER_READ_INVALID  0xFFFFU

void Encoder_Init(void);

/* ISR/foreground SSI read. status_out may be NULL. */
uint8_t Encoder_ReadRawFast(uint16_t *raw_out);
uint8_t Encoder_ReadFrame(uint16_t *raw_out, uint8_t *status_out);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */

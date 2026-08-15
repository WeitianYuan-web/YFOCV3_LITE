#ifndef CALI_H
#define CALI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Cali_Start(void);
uint8_t Cali_RunCommand(uint8_t seq);

#ifdef __cplusplus
}
#endif

#endif /* CALI_H */

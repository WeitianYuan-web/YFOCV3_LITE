#ifndef NODE_H
#define NODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Node_Init(void);
uint8_t Node_GetId(void);
uint8_t Node_ApplyAndReset(uint8_t new_id);
void Node_Service(uint8_t cali_ok);

#ifdef __cplusplus
}
#endif

#endif /* NODE_H */

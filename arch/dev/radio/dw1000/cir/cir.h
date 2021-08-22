#ifndef CIR_H_
#define CIR_H_

#include <stdint.h>

typedef struct {
    uint32_t time;
    uint8_t cir_buf[1016 * 4]; // dw1000_diag.c::201
} cir_measurement;

#endif /* CIR_H_ */

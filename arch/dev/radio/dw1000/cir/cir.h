#ifndef CIR_H_
#define CIR_H_

#include <stdint.h>

#define DW1000_CIR_LEN_PRF64 1016
#define DW1000_CIR_MAX_LEN DW1000_CIR_LEN_PRF64

typedef struct {
    int time;
    uint8_t cir_buf[DW1000_CIR_MAX_LEN * 4]; // dw1000_diag.c::201
} cir_measurement;

typedef struct {
    uint16_t cir_first_half[DW1000_CIR_MAX_LEN];
    uint16_t cir_second_half[DW1000_CIR_MAX_LEN];
} cir_packet;

void send_cir_measurement(cir_measurement *);

#endif /* CIR_H_ */

#include "cir.h"
#include "contiki.h"
#include <complex.h>
#include <stdio.h>
#include <string.h>

extern struct process cir_transmitter_process;
extern process_event_t cir_ready_ev;

static uint8_t abs[1024];
void send_cir_measurement(cir_measurement *cir_m)
{

    int16_t real = 0;
    int16_t imag = 0;

    // TODO: move to application
    int j = 0;
    for (uint16_t i = 1; i < DW1000_CIR_MAX_LEN * 4; i += 4)
    {
        real = (int16_t)(cir_m->cir_buf[i + 1] & 0x00FF) << 8 | (cir_m->cir_buf[i] & 0x00FF);
        imag = (int16_t)(cir_m->cir_buf[i + 3] & 0x00FF) << 8 | (cir_m->cir_buf[i + 2] & 0x00FF);
        double complex comp = real + imag * I;

        abs[j++] = (uint16_t)cabs(comp) >> 4;
    }
    // cir_packet packet;

    // memcpy(packet.cir_first_half, abs, 1023);
    // memcpy(packet.cir_second_half, &abs[512], 512);
    process_post(&cir_transmitter_process, cir_ready_ev, abs);
}

/*
 * Copyright (c) 2017, University of Trento.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file
 *      Contiki DW1000 Advanced Device Diagnostics
 *
 * \author
 *      Davide Vecchia <davide.vecchia@unitn.it>
 */

#include "deca_regs.h"
#include "deca_device_api.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "dw1000-diag.h"
#include "dw1000-config.h"
#include "rtimer.h"
#include "cir/cir.h"

#include "sys/log.h"
#define LOG_MODULE "UWB-TEST"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Reads raw diagnostic data from the radio and computes the received signal
 * power levels for the first path and for the overall transmission according
 * to the DW1000 User Manual (4.7.1 "Estimating the signal power in the first
 * path" and 4.7.2 "Estimating the receive signal power").
 * Requires the current configuration to compute PRF and SFD corrections.
 * Results are stored in the dw1000_diagnostics_t structure.
 *
 * False is returned if power levels could not be computed.
 */
bool
dw1000_diagnostics(dw1000_diagnostics_t *d, const dwt_config_t* config) {

  /* Read diagnostics data */
  dwt_readdiagnostics(&d->dwt_rxdiag);

  /* Get non-saturated preamble accumulator count */
  d->pac_nonsat = dwt_read16bitoffsetreg(DRX_CONF_ID, 0x2C);

  /* Compute corrected preamble counter (used for CIR power adjustment) */
  if(d->dwt_rxdiag.rxPreamCount == d->pac_nonsat) {
    uint16_t sfd_correction = (config->dataRate == DWT_BR_110K) ? 64 : 8;
    d->pac_correction = pow(d->dwt_rxdiag.rxPreamCount - sfd_correction, 2);
  }
  else {
    d->pac_correction = pow(d->dwt_rxdiag.rxPreamCount, 2);
  }

  /* Compute the CIR power level, corrected by PAC value */
  d->cir_pwr = d->dwt_rxdiag.maxGrowthCIR * pow(2, 17) / d->pac_correction;

  /* Compute RX power and First-Path RX power */
  d->fp_raw = (pow(d->dwt_rxdiag.firstPathAmp1, 2) + pow(d->dwt_rxdiag.firstPathAmp2, 2) + pow(d->dwt_rxdiag.firstPathAmp3, 2)) / d->pac_correction;
  if(d->cir_pwr != 0 && d->fp_raw != 0) {
    float prf_correction = (config->prf == DWT_PRF_64M) ? 121.74 : 113.77;
    d->rx_pwr = 10 * log10(d->cir_pwr) - prf_correction;
    d->fp_pwr = 10 * log10(d->fp_raw) - prf_correction;
  }
  else {
    d->rx_pwr = 0;
    d->fp_pwr = 0;
    return false;
  }

  return true;
}

/* Print diagnostics results. */
void
dw1000_print_diagnostics(dw1000_diagnostics_t *d) {
  printf("DW1000 Diagnostics pwr:%.2f fp_pwr:%.2f fp:%u(%u,%u,%u) cir_pwr(raw):%f(%u) pac(nonsat):%u(%u) max_noise:%u std_noise:%u\n",
    d->rx_pwr,
    d->fp_pwr,
    d->dwt_rxdiag.firstPath,
    d->dwt_rxdiag.firstPathAmp1,
    d->dwt_rxdiag.firstPathAmp2,
    d->dwt_rxdiag.firstPathAmp3,
    d->cir_pwr,
    d->dwt_rxdiag.maxGrowthCIR,
    d->dwt_rxdiag.rxPreamCount,
    d->pac_nonsat,
    d->dwt_rxdiag.maxNoise,
    d->dwt_rxdiag.stdNoise
  );
}
#define SPI_READ_LIMIT 128
typedef uint32_t dw1000_cir_sample_t;
#define DW1000_CIR_LEN_PRF16 992
#define DW1000_CIR_LEN_PRF64 1016
#define DW1000_CIR_SAMPLE_SIZE (sizeof(dw1000_cir_sample_t))
#define DW1000_CIR_MAX_LEN DW1000_CIR_LEN_PRF64

typedef struct {
  int16_t real;
  int16_t imag;
} dw1000_cir_sample_t2;

uint16_t dw1000_read_cir(uint16_t s1, uint16_t n_samples, uint8_t* samples) {
  uint16_t max_samples = (dw1000_get_current_cfg()->prf == DWT_PRF_64M) ?
                           DW1000_CIR_LEN_PRF64 :
                           DW1000_CIR_LEN_PRF16;

  if (s1 >= max_samples) {
    printf("Invalid index");
    return 0;
  }

  // adjusting the length to read w.r.t. the accumulator tail size
  if (s1 + n_samples >= max_samples) {
    n_samples = max_samples - s1;
  }

  uint16_t start_byte_idx = s1 * DW1000_CIR_SAMPLE_SIZE;
  uint16_t len_bytes      = n_samples * DW1000_CIR_SAMPLE_SIZE;

  uint16_t read_idx  = start_byte_idx;
  uint8_t* write_pos = (uint8_t*)&samples[1]; // we begin from index 1

#if (SPI_READ_LIMIT > 0)
  uint16_t read_bytes = 0;
  while (read_bytes < len_bytes) {
    uint16_t chunk_size = len_bytes - read_bytes;
    if (chunk_size > SPI_READ_LIMIT) {
      chunk_size = SPI_READ_LIMIT;
    }

    // we need to save one byte from the previous chunk because dwt_readaccdata() always
    // writes zero to the first byte of the current chunk
    uint8_t save_byte = *(write_pos-1);
    dwt_readaccdata(write_pos-1, chunk_size + 1, read_idx);
    *(write_pos-1) = save_byte;
    read_bytes += chunk_size;
    read_idx += chunk_size;
    write_pos += chunk_size;
  }
#else
  dwt_readaccdata(write_pos-1, len_bytes + 1, read_idx);
#endif
  //samples[0].u32 = s1;

  return n_samples;
}
#define MAX_CIR DW1000_CIR_MAX_LEN*4
void print_cir_buf(uint8_t *buf, uint16_t n_samples) {
 /* dw1000_cir_sample_t2 *buf2 = (dw1000_cir_sample_t2 *)buf;
  uint32_t test[1015];
  uint64_t start = RTIMER_NOW();
  for(uint16_t i=0;i<1015;i++) {
    test[i] = (uint32_t) sqrtf(powf(buf2->real,2) + powf(buf2->real,2));
  }

  uint64_t end = RTIMER_NOW();
  uint64_t time = (end - start)*1000;
  time = time/RTIMER_ARCH_SECOND;
  printf("process time %lu \n",(uint32_t)time);
  printf(" \n %lu \n",test[0]);*/

  printf("[");
  for(uint16_t k = 0; k<MAX_CIR; k++) {
    watchdog_periodic();
      printf("\"%02X\"\n",buf[k]);
      if(k!=MAX_CIR-1) printf(",");
      //watchdog_periodic();
  }
  printf("]");
}



uint16_t dw1000_tug_print_cir() {
  uint8_t cir_buf[DW1000_CIR_MAX_LEN*4];
  // print all from the beginning
  dw1000_read_cir(0, DW1000_CIR_MAX_LEN,cir_buf);
  print_cir_buf(&cir_buf[1],DW1000_CIR_MAX_LEN*4);
  return DW1000_CIR_MAX_LEN*4;//dw1000_print_cir_samples_from_radio(0, DW1000_CIR_MAX_LEN); // DW1000_CIR_MAX_LEN
}

void dw1000_tug_print_payload(){
  const size_t buf_len = 128;
  uint8_t buf[buf_len];
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

  //if(frame_len > buf_len) return;
  //dwt_readrxdata(buf, frame_len, 0);
  dwt_readrxdata(buf, buf_len, 0);

  printf("[");
  for(uint8_t i = 0; i < frame_len; i++){
    printf("\"%02X\"", buf[i]);
    if(i != frame_len - 1) printf(",");
  }
  printf("]");

}

cir_measurement cir_m;
void log_cir_measurement(dw1000_dbg_cir_t *debug)
{
    dw1000_read_cir(0, DW1000_CIR_MAX_LEN, cir_m.cir_buf);
    cir_m.time = 255;
    send_cir_measurement(&cir_m);
}

/*
 * Copyright (c) 2018, University of Trento.
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

/*
 * \file
 *    CIR Printing Functions
 *
 * \author
 *    Pablo Corbalan <p.corbalanpelegrin@unitn.it>
 */


#include "dw1000-cir.h"
#include "dw1000-config.h"
#include "contiki.h"
#include "deca_regs.h"
#include "deca_device_api.h"
/*---------------------------------------------------------------------------*/
#include <stdio.h>
/*---------------------------------------------------------------------------*/

/* Number of samples in the accumulator register depending on PRF */
/* Each sample is formed by a 16-bit real + 16-bit imaginary number */
#define ACC_LEN_PRF16 (992*4)
#define ACC_LEN_PRF64 (1016*4)


#define ACC_READ_STEP (128)

/*---------------------------------------------------------------------------*/
static uint8_t acc[ACC_READ_STEP + 1] = {0};
/*---------------------------------------------------------------------------*/

void
print_cir(void)
{
  uint16_t chunk_len = 0;
  uint16_t acc_len_bytes = (dw1000_get_current_cfg()->prf == DWT_PRF_64M) ? 
                           ACC_LEN_PRF64 : 
                           ACC_LEN_PRF16;

  printf("Acc Data [%d]: ", acc_len_bytes);
  for(uint16_t j = 0; j < acc_len_bytes; j = j + ACC_READ_STEP) {
    /* Select the number of bytes to read from the accummulator */
    chunk_len = ACC_READ_STEP;
    if (j + ACC_READ_STEP > acc_len_bytes) {
      chunk_len = acc_len_bytes - j;
    }
    dwt_readaccdata(acc, chunk_len + 1, j);

    for(uint16_t k = 1; k < chunk_len + 1; k++) {
      printf("%02x", acc[k]);
    }
  }
  printf("\n");
}
/*---------------------------------------------------------------------------*/
void
print_cir_samples(uint16_t s1, uint16_t len)
{
  uint16_t chunk_len = 0;
  
  uint16_t acc_len_bytes = (dw1000_get_current_cfg()->prf == DWT_PRF_64M) ? 
                           ACC_LEN_PRF64 : 
                           ACC_LEN_PRF16;

  uint16_t nbytes = (len < ACC_READ_STEP) ? len : ACC_READ_STEP;
  uint16_t max_bytes = (s1 + len < acc_len_bytes) ? (s1 + len) : acc_len_bytes;

  printf("Acc Data: ");
  for(uint16_t j = s1; j < max_bytes; j = j + nbytes) {
    /* Select the number of bytes to read from the accummulator */
    chunk_len = nbytes;
    if (j + nbytes > max_bytes) {
      chunk_len = max_bytes - j;
    }
    dwt_readaccdata(acc, chunk_len + 1, j);

    for(uint16_t k = 1; k < chunk_len + 1; k++) {
      printf("%02x", acc[k]);
    }
  }
  printf("\n");
}
/*---------------------------------------------------------------------------*/
void
print_readable_cir(void)
{
  uint16_t chunk_len = 0;

  uint16_t acc_len_bytes = (dw1000_get_current_cfg()->prf == DWT_PRF_64M) ? 
                           ACC_LEN_PRF64 : 
                           ACC_LEN_PRF16;

  int16_t a = 0; /* Real part */
  int16_t b = 0; /* Imaginary part */

  printf("Acc Data [%d]: ", acc_len_bytes);
  for(int j = 0; j < acc_len_bytes; j = j + ACC_READ_STEP) {
    /* Select the number of bytes to read from the accummulator */
    chunk_len = ACC_READ_STEP;
    if (j + ACC_READ_STEP > acc_len_bytes) {
      chunk_len = acc_len_bytes - j;
    }
    dwt_readaccdata(acc, chunk_len + 1, j);

    /* Print the bytes read as complex numbers */
    for(int k = 1; k < chunk_len + 1; k = k + 4) {
      a = (int16_t) (((acc[k + 1] & 0x00FF) << 8) | (acc[k] & 0x00FF));
      b = (int16_t) (((acc[k + 3] & 0x00FF) << 8) | (acc[k + 2] & 0x00FF));
      if(b >= 0) {
        printf("%d+%dj,", a, b);
      } else {
        printf("%d%dj,", a, b);
      }
    }
  }
  printf("\n");
}
/*---------------------------------------------------------------------------*/

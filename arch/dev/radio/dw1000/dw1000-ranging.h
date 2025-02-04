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
 *      Contiki DW1000 Driver ranging module
 *
 * \author
 *      Timofei Istomin <tim.ist@gmail.com>
 */

#include "dw1000.h"
#include "dw1000-arch.h"
#include "os/net/linkaddr.h"
#include "contiki-conf.h"

#ifndef DW1000_RANGING_H
#define DW1000_RANGING_H

/*---------------------------------------------------------------------------*/

#ifdef DW1000_CONF_COMPENSATE_BIAS
#define DW1000_COMPENSATE_BIAS DW1000_CONF_COMPENSATE_BIAS
#else
#define DW1000_COMPENSATE_BIAS 1
#endif

/* (Re)initialise the ranging module.
 *
 * Needs to be called before issuing or serving ranging requests and
 * after changing radio parameters. */
void dw1000_ranging_init();

/*---------------------------------------------------------------------------*/
/* Callback to process ranging good frame events */
void
dw1000_rng_ok_cb(const dwt_cb_data_t *cb_data);

/* Callback to process tx confirmation events */
void
dw1000_rng_tx_conf_cb(const dwt_cb_data_t *cb_data);
/*---------------------------------------------------------------------------*/

bool dw1000_range_with(linkaddr_t *lladdr, dw1000_rng_type_t type);
bool dw1000_is_ranging();
void dw1000_range_reset();
void dw1000_ranging_enable_cir_print(bool enable);

extern process_event_t ranging_event;

typedef struct {
  int status;       /* 1=SUCCESS, 0=FAIL */
  double distance;
  double raw_distance;
} ranging_data_t;

#endif /* DW1000_RANGING_H */

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
 *      Contiki DW1000 Driver Source File
 *
 * \author
 *      Pablo Corbalan <p.corbalanpelegrin@unitn.it>
 *      Timofei Istomin <tim.ist@gmail.com>
 */

#include "dw1000.h"
#include "dw1000-diag.h"
#include "dw1000-arch.h"
#include "dw1000-ranging.h"
#include "dw1000-config.h"
#include "dw1000-shared-state.h"
#include "net/packetbuf.h"
//#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "leds.h" /* To be removed after debugging */
#include <stdbool.h>
#include "dev/watchdog.h"
/*---------------------------------------------------------------------------*/
#include "deca_device_api.h"
#include "deca_regs.h"
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "DWD"
#define LOG_LEVEL LOG_LEVEL_DBG


#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#undef LEDS_TOGGLE
#if DW1000_DEBUG_LEDS
#define LEDS_TOGGLE(x) leds_toggle(x)
#else
#define LEDS_TOGGLE(x)
#endif

/*---------------------------------------------------------------------------*/
/* Configuration constants */
/*#define DW1000_RX_AFTER_TX_DELAY 60 */
#define DW1000_RX_AFTER_TX_DELAY 0
/*---------------------------------------------------------------------------*/

#if DEBUG
typedef enum {
  FRAME_RECEIVED = 1,
  RECV_TO,
  RECV_ERROR,
  TX_SUCCESS,
} dw1000_event_t;
static dw1000_event_t dw_dbg_event;
static uint32_t radio_status;
#endif

/* Static variables */
static uint16_t data_len; /* received data length (payload without CRC) */
static bool frame_pending;
static bool frame_uploaded;
static bool auto_ack_enabled;
static bool wait_ack_txdone;

static volatile bool tx_done; /* flag indicating the end of TX */

/* Variables exported to other modules of the radio driver */
bool dw1000_is_sleeping; /* true when the radio is in DEEP SLEEP mode */

/* RSSI/FPPL  */
/* Diag debug enable */
#if DW1000_READ_RXDIAG
#include <math.h> // log()
/* read complex values from the ACCUMULATOR memory that represent the CIR
 * of the RF channel. Each tap represents 1ns sample interval.
 * 19.2m ~ 1 + 4 * 64 byte = 257 (1 dummy byte + 4 byte per tap. 2real, 2imag)
 * --> DW1000_Software_API_Guide_rev2p4 c.5.54
 */
uint8_t cir_buffer[CIR_BUF_LEN];

#endif /* DW1000_READ_RXDIAG */
dwt_rxdiag_t dw1000_diag;
static volatile radio_value_t dw1000_last_lqi;
static volatile radio_value_t dw1000_last_rssi;
static volatile radio_value_t dw1000_last_fppl;
static volatile radio_value_t dw1000_last_rxtime;

/*---------------------------------------------------------------------------*/
PROCESS(dw1000_process, "DW1000 driver");
#if DEBUG
PROCESS(dw1000_dbg_process, "DW1000 dbg process");
#endif
/*---------------------------------------------------------------------------*/
/* Declaration of static radio callback functions.
 * NOTE: For all events, corresponding interrupts are cleared and necessary
 * resets are performed. In addition, in the RXFCG case, received frame
 * information and frame control are read before calling the callback. If
 * double buffering is activated, it will also toggle between reception
 * buffers once the reception callback processing has ended.
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);
/*---------------------------------------------------------------------------*/
/* DW1000 Radio Driver Static Functions */
static int dw1000_init(void);
static int dw1000_prepare(const void *payload, unsigned short payload_len);
static int dw1000_transmit(unsigned short transmit_len);
static int dw1000_send(const void *payload, unsigned short payload_len);
static int dw1000_radio_read(void *buf, unsigned short buf_len);
static int dw1000_channel_clear(void);
static int dw1000_receiving_packet(void);
static int dw1000_pending_packet(void);
static int dw1000_on(void);
static int dw1000_off(void);
static radio_result_t dw1000_get_value(radio_param_t param, radio_value_t *value);
static radio_result_t dw1000_set_value(radio_param_t param, radio_value_t value);
static radio_result_t dw1000_get_object(radio_param_t param, void *dest, size_t size);
static radio_result_t dw1000_set_object(radio_param_t param, const void *src, size_t size);
/*---------------------------------------------------------------------------*/
/* Callback to process RX good frame events */
static void
rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  // dw100_dbg_cir_t debug;
  // static uint16_t sleep = 0;
  // debug.status_reg = cb_data->status;
  // debug.rx_on_delay = sleep;


  /*LEDS_TOGGLE(LEDS_GREEN); */
#if DW1000_RANGING_ENABLED
  if(cb_data->rx_flags & DWT_CB_DATA_RX_FLAG_RNG) {
    dw1000_rng_ok_cb(cb_data);
   // printf(" RX time %d fppl %d rssi %d %u %u \n",dw1000_last_rxtime,dw1000_last_fppl,dw1000_last_rssi,dw1000_diag.maxGrowthCIR, dw1000_diag.firstPath);
      #if DW1000_READ_RXDIAG
        dwt_readdiagnostics(&dw1000_diag);
        dw1000_last_rxtime = RTIMER_NOW();
        
        /* rssi [dBm] = 10*log10((C*2^17)/N^2)-A
          * C = CIR_PWR (reg 0x12)
          * A = constant -> 113.77 for 16MHz PRF, 121.74 for 64MHz PRF
          * N = RXPACC (reg 0x10)
          */
        dw1000_last_lqi = 107;
        float C = (float)(((uint32_t)dw1000_diag.maxGrowthCIR) << 17);
        float A = (DW1000_CONF_PRF == DWT_PRF_64M) ? 113.77f : 121.74f;
        uint16_t N = dw1000_diag.rxPreamCount;
        dw1000_last_rssi = 10.0f * (log10f(C) -
                log10f(N * N))
                - A;

        /*FPPL = 10*log10((F1^1 + F2^2 + F3^2)/N^2) - A
        * F1 = FP_AMPL1 (reg 0x15)
        * F2 = FP_AMPL2 (reg 0x12)
        * F3 = FP_AMPL3 (reg 0x12)
        */
        float F1 = (float)((uint32_t)dw1000_diag.firstPathAmp1);
        float F2 = (float)((uint32_t)dw1000_diag.firstPathAmp2);
        float F3 = (float)((uint32_t)dw1000_diag.firstPathAmp3);
        dw1000_last_fppl = 10.0f * (log10f(F1 * F1 + F2 * F2 + F3 * F3) -
                log10f(N * N))
                - A;
    #endif

    log_cir_measurement(NULL);
    // dw1000_tug_print_diagnostics_json(true, false,&debug);
    return;
  }
  /* got a non-ranging packet: reset the ranging module if */
  /* it was in the middle of ranging */
  dw1000_range_reset();
#endif

  data_len = cb_data->datalength - DW1000_CRC_LEN;
  /* Set the appropriate event flag */
  frame_pending = true;
#if DEBUG
  radio_status = cb_data->status;
#endif
  /* if we have auto-ACKs enabled and an ACK was requested, */
  /* don't signal the reception until the TX done interrupt */
  if(auto_ack_enabled && (cb_data->status & SYS_STATUS_AAT)) {
    /*leds_on(LEDS_ORANGE); */
    wait_ack_txdone = true;
  } else {
    wait_ack_txdone = false;
    process_poll(&dw1000_process);
  }
}
/*---------------------------------------------------------------------------*/
/* Callback to process RX timeout events */
static void
rx_to_cb(const dwt_cb_data_t *cb_data)
{
#if DW1000_RANGING_ENABLED
  dw1000_range_reset();
#endif
#if DEBUG
  dw_dbg_event = RECV_TO;
  radio_status = cb_data->status;
  process_poll(&dw1000_dbg_process);
#endif
  dw1000_on();

  LEDS_TOGGLE(LEDS_YELLOW);
  /* Set LED PC7 */
}
/*---------------------------------------------------------------------------*/
/* Callback to process RX error events */
// static void
// rx_err_cb(const dwt_cb_data_t *cb_data)
// {
// #if DW1000_RANGING_ENABLED
//   dw1000_range_reset();
// #endif
// #if DEBUG
//   dw_dbg_event = RECV_ERROR;
//   radio_status = cb_data->status;
//   process_poll(&dw1000_dbg_process);
// #endif
//   dw1000_on();
//
//   /* Set LED PC8 */
//   /*LEDS_TOGGLE(LEDS_RED); // not informative with frame filtering */
// }
static void
rx_err_cb(const dwt_cb_data_t *cb_data)
{
  // dw1000_dbg_cir_t debug;
  static uint16_t sleep = 5;
  radio_status = cb_data->status;
  // debug.status_reg = cb_data->status;
  // debug.rx_on_delay = sleep;
  //deca_sleep(1);
  //uint64_t time = RTIMER_NOW();
  //time = time*1000;
  //time = time/RTIMER_ARCH_SECOND;
  //printf("Err Callback  %lu \n",(uint32_t)time);
  //dwt_setrxtimeout(0);
  if(radio_status & (SYS_STATUS_RXSFDTO)) { //| SYS_STATUS_RXRFSL | SYS_STATUS_RXPHE
    dwt_setdelayedtrxtime(dwt_readsystimestamphi32()+1872004); // 15E-3/(15.65E-12 * 2^9) = 1248003 delay for 
    dwt_rxenable(DWT_START_RX_DELAYED);
    deca_sleep(sleep);
    // dw1000_tug_print_diagnostics_json(true, false,&debug);

    dwt_forcetrxoff();
    dwt_rxreset();
    //deca_sleep(1000);
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  } else {
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
}


/*---------------------------------------------------------------------------*/
/* Callback to process TX confirmation events */
static void
tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  /* Set LED PC9 */
  /*LEDS_TOGGLE(LEDS_ORANGE); */

#if DW1000_RANGING_ENABLED
  dw1000_rng_tx_conf_cb(cb_data);
#endif

  tx_done = 1; /* to stop waiting in dw1000_transmit() */
  frame_uploaded = 0;

  /*if we are sending an auto ACK, signal the frame reception here */
  if(wait_ack_txdone) {
    wait_ack_txdone = false;
    process_poll(&dw1000_process);
  }
}
/*---------------------------------------------------------------------------*/

static int
dw1000_init(void)
{
  PRINTF("DW1000 driver init\n");
  /* Initialize arch-dependent DW1000 */
  dw1000_arch_init();

  /* Set the default configuration */
  dw1000_reset_cfg();

  /* Print the current configuration */
#ifndef DW1000_SKIP_PRINT_CONFIG
  dw1000_print_cfg();
#endif

  /* Configure DW1000 GPIOs to show TX/RX activity with the LEDs */
#if DW1000_DEBUG_LEDS
  dwt_setleds(DWT_LEDS_ENABLE);
#endif

  auto_ack_enabled = false;

#if DW1000_FRAMEFILTER == 1
  dw1000_set_value(RADIO_PARAM_RX_MODE, RADIO_RX_MODE_ADDRESS_FILTER);
#endif /* DW1000_FRAMEFILTER */

  /* Set the DW1000 ISR */
  dw1000_set_isr(dwt_isr);

  /* Register TX/RX callbacks. */
  dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO |
                   DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT |
                   DWT_INT_ARFE, 1);

#if DW1000_RANGING_ENABLED
  dw1000_ranging_init();
#endif

  /* Configure deep sleep mode */
  /* NOTE: this is only used if the application actually calls the necessary
   * functions to put the radio in deep sleep mode and wake it up */
  dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);

  /* Start DW1000 process */
  process_start(&dw1000_process, NULL);
#if DEBUG
  process_start(&dw1000_dbg_process, NULL);
#endif /* DEBUG */

  dw1000_last_lqi = 110;
  dw1000_last_rssi = 0;
  dw1000_last_fppl = 0;
  dw1000_last_rxtime = 0;

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t frame_len;

  if (dw1000_is_sleeping) {
    PRINTF("Err: TX prepare requested while sleeping\n");
    return RADIO_TX_ERR;
  }

  frame_uploaded = 0;
#if DW1000_RANGING_ENABLED
  if(dw1000_is_ranging()) {
    return RADIO_TX_ERR;
  }
#endif

  frame_len = payload_len + DW1000_CRC_LEN;
  
  if (frame_len > 127) {
    PRINTF("Err: TX len %u\n", frame_len);
    return RADIO_TX_ERR;
  }

  /* Write frame data to DW1000 and prepare transmission */
  dwt_writetxdata(frame_len, (uint8_t *)payload, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(frame_len, 0, 0); /* Zero offset in TX buffer, no ranging. */
  /* TODO: check the return status of the operations above */
  frame_uploaded = 1;
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_transmit(unsigned short transmit_len)
{
  int ret;

  if (dw1000_is_sleeping) {
    PRINTF("Err: transmit requested while sleeping\n");
    return RADIO_TX_ERR;
  }
  
  if (!frame_uploaded) {
    PRINTF("Err: transmit without prepare\n");
    return RADIO_TX_ERR;
  }
  int8_t irq_status = dw1000_disable_interrupt();
#if DW1000_RANGING_ENABLED
  if(dw1000_is_ranging()) {
    dw1000_enable_interrupt(irq_status);
    return RADIO_TX_ERR;
  }
#endif
  /* Switch off radio before setting it to transmit
   * It also clears pending interrupts */
  dwt_forcetrxoff();

  /* Radio starts listening certain delay (in UWB microseconds) after TX */
  dwt_setrxaftertxdelay(DW1000_RX_AFTER_TX_DELAY);

  tx_done = false;
  wait_ack_txdone = false;

  /* Start transmission, indicating that a response is expected so that reception
   * is enabled automatically after the frame is sent and the delay set by
   * dwt_setrxaftertxdelay() has elapsed. */
  ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  dw1000_enable_interrupt(irq_status);

  if(ret != DWT_SUCCESS) {
    return RADIO_TX_ERR;
  }

  watchdog_periodic();
  while(!tx_done) {
    /* do nothing, could go to LPM mode */
    //asm("");
    clock_delay_usec(1);
    /* TODO: add a timeout */
  }
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_send(const void *payload, unsigned short payload_len)
{
  if (dw1000_is_sleeping) {
    PRINTF("Err: send requested while sleeping\n");
    return RADIO_TX_ERR;
  }

  if(0 == dw1000_prepare(payload, payload_len)) {
    return dw1000_transmit(payload_len);
  } else {
    return RADIO_TX_ERR;
  }
}
/*---------------------------------------------------------------------------*/
static int
dw1000_radio_read(void *buf, unsigned short buf_len)
{
  if (dw1000_is_sleeping)
    return 0;

  if(!frame_pending) {
    return 0;
  }
  dwt_readrxdata(buf, buf_len, 0);
  frame_pending = false;
  return buf_len;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_channel_clear(void)
{
  if (dw1000_is_sleeping)
    return 0;

#if DW1000_RANGING_ENABLED
  if(dw1000_is_ranging()) {
    return 0;
  }
#endif /* DW1000_RANGING_ENABLED */

  if(wait_ack_txdone) {
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_receiving_packet(void)
{
  /* TODO: fix this by checking the actual radio status */
  if(wait_ack_txdone) {
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_pending_packet(void)
{
  return frame_pending;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_on(void)
{
  if (dw1000_is_sleeping) {
    PRINTF("Err: radio ON requested while sleeping\n");
    return RADIO_RESULT_ERROR;
  }

  /* Enable RX */
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_off(void)
{
  if (dw1000_is_sleeping) {
    PRINTF("Warning: radio OFF requested while sleeping\n");
    return 0;
  }

  /* Turn off the transceiver */
  int8_t irq_status = dw1000_disable_interrupt();
#if DW1000_RANGING_ENABLED
  dw1000_range_reset(); /* In case we were ranging */
#endif
  dwt_forcetrxoff();
  wait_ack_txdone = 0;
  dw1000_enable_interrupt(irq_status);

  return 0;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_get_value(radio_param_t param, radio_value_t *value)
{
  if (dw1000_is_sleeping)
    return RADIO_RESULT_ERROR;  

  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_set_value(radio_param_t param, radio_value_t value)
{
  if (dw1000_is_sleeping)
    return RADIO_RESULT_ERROR;  

  switch(param) {
  case RADIO_PARAM_PAN_ID:
    dwt_setpanid(value & 0xFFFF);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_16BIT_ADDR:
    LOG_DBG("Set Short Address: 0x%02x%02x \n",(uint8_t)(value>>8),(uint8_t)value);
    dwt_setaddress16(value & 0xFFFF);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    if(value & RADIO_RX_MODE_ADDRESS_FILTER) {
      dwt_enableframefilter(DWT_FF_COORD_EN | DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN | DWT_FF_MAC_EN);
#if DW1000_AUTOACK
      /* Auto-ack is only possible if frame filtering is activated */
      dwt_enableautoack(DW1000_AUTOACK_DELAY);
      auto_ack_enabled = true;
#endif
    } else {
      dwt_enableframefilter(DWT_FF_NOTYPE_EN);
      auto_ack_enabled = false;
    }
    return RADIO_RESULT_OK;
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_get_object(radio_param_t param, void *dest, size_t size)
{
  if (dw1000_is_sleeping)
    return RADIO_RESULT_ERROR;  

  if(param == RADIO_PARAM_64BIT_ADDR) {
    if(size != 8 || dest == NULL) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    uint8_t little_endian[8];
    int i;
    dwt_geteui(little_endian);

    for(i = 0; i < 8; i++) {
      ((uint8_t*)dest)[i] = little_endian[7 - i];
    }
    return RADIO_RESULT_OK;
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_set_object(radio_param_t param, const void *src, size_t size)
{
  if (dw1000_is_sleeping)
    return RADIO_RESULT_ERROR;  

  if(param == RADIO_PARAM_64BIT_ADDR) {
    if(size != 8 || src == NULL) {
      return RADIO_RESULT_INVALID_VALUE;
    }

    uint8_t little_endian[8];
    int i;

    for(i = 0; i < 8; i++) {
      little_endian[i] = ((uint8_t *)src)[7 - i];
    }
    dwt_seteui(little_endian);

    return RADIO_RESULT_OK;
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw1000_process, ev, data)
{
  PROCESS_BEGIN();

  LOG_INFO("dw1000_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

#if DEBUG
    uint32_t r1 = radio_status;
    LOG_DBG("RX OK: %02x %02x %02x %02x\n",
           (uint8_t)(r1 >> 24), (uint8_t)(r1 >> 16),
           (uint8_t)(r1 >> 8), (uint8_t)r1);
#endif

    if(!frame_pending) {
      /* received a frame but it was already read (e.g. ACK) */
      /* re-enable rx */
      dw1000_on();
      continue;
    }

    if(data_len > PACKETBUF_SIZE) {
      frame_pending = false;
      dw1000_on();
      continue; /* packet is too big, drop it */
    }

    /* Clear packetbuf to avoid having leftovers from previous receptions */
    packetbuf_clear();

    /* Copy the received frame to packetbuf */
    dw1000_radio_read(packetbuf_dataptr(), data_len);
    packetbuf_set_datalen(data_len);

    /* Re-enable RX to keep listening */
    dw1000_on();
    /*PRINTF("dw1000_process: calling recv cb, len %d\n", data_len); */
    NETSTACK_MAC.input();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
const struct radio_driver dw1000_driver =
{
  dw1000_init,
  dw1000_prepare,
  dw1000_transmit,
  dw1000_send,
  dw1000_radio_read,
  dw1000_channel_clear,
  dw1000_receiving_packet,
  dw1000_pending_packet,
  dw1000_on,
  dw1000_off,
  dw1000_get_value,
  dw1000_set_value,
  dw1000_get_object,
  dw1000_set_object
};
/*---------------------------------------------------------------------------*/
/* Functions to put DW1000 into deep sleep mode and wake it up */
void
dw1000_sleep(void)
{
  if (dw1000_is_sleeping)
    return;

  dw1000_disable_interrupt(); // disable and keep disabled until wakeup

#if DW1000_RANGING_ENABLED
  dw1000_range_reset(); /* In case we were ranging */
#endif

  frame_pending   = 0;
  frame_uploaded  = 0; // frame is not preserved during sleep
  wait_ack_txdone = 0;
  dwt_entersleep();
  dw1000_is_sleeping = 1;
}
/*---------------------------------------------------------------------------*/
int
dw1000_wakeup(void)
{
  dw1000_disable_interrupt();
  
  if(dwt_readdevid() != DWT_DEVICE_ID) { // Device was in deep sleep (the first read fails)
    dw1000_arch_wakeup_nowait();
    deca_sleep(5); // need to sleep 5 ms to let crystal stabilise
    
    if(dwt_readdevid() != DWT_DEVICE_ID) {
      dw1000_is_sleeping = 1;
      return DWT_ERROR;
    }
  } 

  dw1000_is_sleeping = 0;
  
  /* Restore antenna delay values for ranging */
  dw1000_restore_ant_delay();
  
#if LINKADDR_SIZE == 8
  // DW1000 does not preserve the extended address while sleeping
  NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, linkaddr_node_addr.u8, 8);
#endif

  dw1000_enable_interrupt(1);
#if DW1000_DEBUG_LEDS
  dwt_setleds(DWT_LEDS_ENABLE);
#endif
  return DWT_SUCCESS;
}
/*---------------------------------------------------------------------------*/
bool
range_with(linkaddr_t *dst, dw1000_rng_type_t type)
{
#if DW1000_RANGING_ENABLED
  if (dw1000_is_sleeping)
    return false;
 
  wait_ack_txdone = 0;
  frame_pending   = 0;
  frame_uploaded  = 0;
  return dw1000_range_with(dst, type);
#else
LOG_INFO("R2 \n");
  return false;
#endif
}
#if DEBUG
PROCESS_THREAD(dw1000_dbg_process, ev, data)
{
  static struct etimer et;
  static uint32_t r1;
  static uint8_t r2;
  PROCESS_BEGIN();
  while(1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_POLL) {
      r1 = radio_status;
      // printf("RX FAIL(%u) %02x %02x %02x %02x\n",
      //        dw_dbg_event, (uint8_t)(r1 >> 24), (uint8_t)(r1 >> 16),
      //        (uint8_t)(r1 >> 8), (uint8_t)r1);
    }
    if(etimer_expired(&et) && !dw1000_is_sleeping) {
      r1 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
      r2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
      printf("*** SYS_STATUS %02x %02x %02x %02x %02x ***\n",
             (uint8_t)(r1 >> 24), (uint8_t)(r1 >> 16), (uint8_t)(r1 >> 8),
             (uint8_t)(r1), r2);
      dw_dbg_event = 0;
    }
  }
  PROCESS_END();
}
#endif /* DEBUG */

radio_value_t uwb_get_rssi(void)
{
  return dw1000_last_rssi;
}

radio_value_t uwb_get_fppl(void)
{
  return dw1000_last_fppl;
}

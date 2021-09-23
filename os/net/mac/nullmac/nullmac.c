/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A MAC protocol that does not do anything.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "net/mac/nullmac/nullmac.h"
#include "net/netstack.h"
#include "net/ipv6/uip.h"
#include "net/ipv6/tcpip.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "sys/cc.h"
#include "lib/random.h"
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "NULLMAC"
#define LOG_LEVEL LOG_LEVEL_NULLNET

/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  //static uint8_t icnt = 0;
  static uint8_t initialized = 0;
  static uint8_t seqno;
  linkaddr_t *addr = (linkaddr_t *)packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(!initialized) {
    initialized = 1;
    /* Initialize the sequence number to a random value as per 802.15.4. */
    seqno = random_rand();
  }
  if(seqno == 0) {
    /* PACKETBUF_ATTR_MAC_SEQNO cannot be zero, due to a pecuilarity
       in framer-802154.c. */
    seqno++;
  }
  NETSTACK_FRAMER.create();
  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);
  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, (const linkaddr_t *)addr);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, seqno++);
  

  LOG_DBG("Send packet to : ");
  LOG_DBG_LLADDR(addr);
  LOG_DBG_(" Data: ");
    //for(uint8_t i = 0; i<20;i++){
    //printf("%02x,",((uint8_t*)packetbuf_hdrptr())[i]);
  //}
  LOG_DBG_("\n");

  uint8_t *hdrptr = packetbuf_hdrptr();
  // TODO: Just a test please fix it!
  hdrptr[0] = 0x61;
  hdrptr[1] = 0x98;

  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());
  NETSTACK_RADIO.transmit(packetbuf_totlen());
}
/*---------------------------------------------------------------------------*/
int parse() {
 int hdr_len;

  hdr_len = NETSTACK_FRAMER.parse();
  if(hdr_len < 0) {
    return hdr_len;
  }

  if(linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER), &linkaddr_node_addr)) {
    LOG_INFO("frame from ourselves\n");
    return FRAMER_FAILED;
  }

  /* TODO anti-reply protection */
  return hdr_len;
}
/*---------------------------------------------------------------------------*/

static void
packet_input(void)
{
  //parse();
  int hdr_len = NETSTACK_FRAMER.parse();
  LOG_INFO("Header len %d \n",hdr_len);

  LOG_INFO("received packet from ");
  LOG_INFO_LLADDR(packetbuf_addr(PACKETBUF_ADDR_SENDER));
  LOG_INFO_(", seqno %u, len %u Data:", packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO), packetbuf_datalen());
  //for(uint8_t i = 0; i<20;i++){
  //  printf("%02x,",((uint8_t*)packetbuf_hdrptr())[i]);
  //}
  LOG_INFO_("\n");
  
  NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  return NETSTACK_RADIO.off();
}
/*---------------------------------------------------------------------------*/
static int
max_payload(void)
{
  int framer_hdrlen;
  radio_value_t max_radio_payload_len;
  radio_result_t res;

  framer_hdrlen = NETSTACK_FRAMER.length();

  res = NETSTACK_RADIO.get_value(RADIO_CONST_MAX_PAYLOAD_LEN,
                                 &max_radio_payload_len);

  if(res == RADIO_RESULT_NOT_SUPPORTED) {
    LOG_ERR("Failed to retrieve max radio driver payload length\n");
    return 0;
  }
/*
  if(framer_hdrlen < 0) {
     Framing failed, we assume the maximum header length 
    framer_hdrlen = CSMA_MAC_MAX_HEADER;
  }*/


  return MIN(max_radio_payload_len, PACKETBUF_SIZE)
    - framer_hdrlen
    - LLSEC802154_PACKETBUF_MIC_LEN();
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
}
/*---------------------------------------------------------------------------*/
const struct mac_driver nullmac_driver = {
  "nullmac",
  init,
  send_packet,
  packet_input,
  on,
  off,
  max_payload,
};
/*---------------------------------------------------------------------------*/

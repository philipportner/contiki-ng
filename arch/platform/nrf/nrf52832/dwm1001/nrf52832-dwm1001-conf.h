/*
 * Copyright (C) 2020 Yago Fontoura do Rosario <yago.rosario@hotmail.com.br>
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup nrf-platforms
 * @{
 *
 * \addtogroup nrf52832-dwm1001
 * @{
 *
 * \file
 *         nRF52832 DWM1001 specific configuration.
 * \author
 *         Yago Fontoura do Rosario <yago.rosario@hotmail.com.br>
 */
/*---------------------------------------------------------------------------*/
#ifndef NRF52832_DK_CONF_H
#define NRF52832_DK_CONF_H
/*---------------------------------------------------------------------------*/
#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO        dw1000_driver
#endif /* NETSTACK_CONF_RADIO */
/*---------------------------------------------------------------------------*/
#define PLATFORM_HAS_UARTE              1
#define PLATFORM_HAS_BUTTON             1
#define PLATFORM_SUPPORTS_BUTTON_HAL    1
/*---------------------------------------------------------------------------*/
#define NRF_BUTTON1_PIN     11
#define NRF_BUTTON1_PORT    0
#define NRF_BUTTON2_PIN     12
#define NRF_BUTTON2_PORT    0
#define NRF_BUTTON3_PIN     24
#define NRF_BUTTON3_PORT    0
#define NRF_BUTTON4_PIN     25
#define NRF_BUTTON4_PORT    0
/*---------------------------------------------------------------------------*/
#define NRF_LED1_PIN        13
#define NRF_LED1_PORT       0
#define NRF_LED2_PIN        14
#define NRF_LED2_PORT       0
#define NRF_LED3_PIN        15
#define NRF_LED3_PORT       0
#define NRF_LED4_PIN        16
#define NRF_LED4_PORT       0
/*---------------------------------------------------------------------------*/
#define LEDS_CONF_COUNT     4
/*---------------------------------------------------------------------------*/
#define NRF_UARTE0_TX_PIN   5
#define NRF_UARTE0_TX_PORT  0
#define NRF_UARTE0_RX_PIN   11
#define NRF_UARTE0_RX_PORT  0
/*---------------------------------------------------------------------------*/
#define DW1000_RST        NRF_GPIO_PIN_MAP(0,24)
#define DW1000_IRQ_EXTI   NRF_GPIO_PIN_MAP(0,19)
#define SPI_CS_PIN        NRF_GPIO_PIN_MAP(0,17)
#define SPI1_SCK_PIN      NRF_GPIO_PIN_MAP(0,16)
#define SPI1_MOSI_PIN     NRF_GPIO_PIN_MAP(0,20)
#define SPI1_MISO_PIN     NRF_GPIO_PIN_MAP(0,18)
#define SPI1_IRQ_PRIORITY 6
#define SPI_INSTANCE      0 
/*---------------------------------------------------------------------------*/
#define NRFX_SPI_ENABLED 1
#define NRFX_SPI0_ENABLED 1
#define NRFX_SPI1_ENABLED 1
/*---------------------------------------------------------------------------*/
#ifndef DW1000_CONF_RX_ANT_DLY
#define DW1000_CONF_RX_ANT_DLY 16455 // TODO: needs calibration
#endif
#ifndef DW1000_CONF_TX_ANT_DLY
#define DW1000_CONF_TX_ANT_DLY 16455 // TODO: needs calibration
#endif
#ifndef DW1000_CONF_FRAMEFILTER
#define DW1000_CONF_FRAMEFILTER 1
#endif
/*---------------------------------------------------------------------------*/
#endif /* NRF52840_DK_CONF_H */
/*---------------------------------------------------------------------------*/
/** 
 * @} 
 * @} 
 */

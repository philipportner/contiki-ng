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
 * \file
 *      Platform implementation for nRF
 * \author
 *      Yago Fontoura do Rosario <yago.rosario@hotmail.com.br>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"

#include "dev/gpio-hal.h"
#include "dev/button-hal.h"
#include "dev/leds.h"
#include "dev/serial-line.h"

#include "random.h"
#include "int-master.h"
#include "sensors.h"
#include "uarte-arch.h"
#include "linkaddr-arch.h"
#include "reset-arch.h"

#include "lpm.h"

extern void board_init(void);

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "NRF"
#define LOG_LEVEL LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/
void
platform_init_stage_one(void)
{
  gpio_hal_init();
  leds_init();
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_two(void)
{
  button_hal_init();

  /* Seed value is ignored since hardware RNG is used. */
  random_init(0x5678);

#if PLATFORM_HAS_UARTE
  uarte_init();
  serial_line_init();
#if BUILD_WITH_SHELL
  uarte_set_input(serial_line_input_byte);
#endif /* BUILD_WITH_SHELL */
#endif /* PLATFORM_HAS_UARTE */
  populate_link_address();

  reset_debug();
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_three(void)
{
  board_init();

  process_start(&sensors_process, NULL);
}
/*---------------------------------------------------------------------------*/
void
platform_idle()
{
  lpm_drop();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */

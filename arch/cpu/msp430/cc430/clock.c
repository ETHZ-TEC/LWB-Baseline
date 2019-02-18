/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 *
 * Author:  Jonas Baechli
 *          Reto Da Forno
 *
 */

#include "contiki.h"

/*---------------------------------------------------------------------------*/
static volatile clock_time_t clock_count;
static volatile clock_time_t clock_secs;
#define MAX_TICKS         (~((clock_time_t)0) / 2)
#define CLOCK_INTERVAL    (RTIMER_EXT_SECOND_LF / CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
void
clock_init()
{
  /* note: rtimer_ext_init() is called in stage one platform init, before this
   * function is called by contiki-main */

  clock_count = 0;
  clock_secs = 0;

#if RTIMER_EXT_CONF_USE_ETIMER
  #if ETIMER_ARCH_TIMER_ID != RTIMER_EXT_LF_2
  #error "wrong rtimer_ext ID for etimer!"
  #endif
  TA1CCR2   = CLOCK_INTERVAL;
  TA1CCTL2 |= CCIE;
#endif /* RTIMER_EXT_CONF_USE_ETIMER */
}
/*---------------------------------------------------------------------------*/
CCIF unsigned long
clock_seconds(void)
{
  return clock_secs;
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  return clock_count;
}
/*---------------------------------------------------------------------------*/
uint16_t
clock_counter(void)
{
  uint16_t t1, t2;
  do {
    t1 = TA1R;
    t2 = TA1R;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
void
clock_delay(unsigned int i)
{
  while(i--) {
    _NOP();
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
clock_ccrint_cb(void)
{
  /* periodic capture compare register interrupt, called from rtimer-ext.c */
  clock_count++;
  /* increment seconds counter if necessary */
  if(clock_count % CLOCK_CONF_SECOND == 0) {
    clock_secs++;
  }
  TA1CCR2 += CLOCK_INTERVAL;
  /* check whether there are etimers ready to be served */
  if(etimer_pending() &&
      (etimer_next_expiration_time() - clock_count - 1) > MAX_TICKS) {
    etimer_request_poll();
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/

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
 * Author:  Reto Da Forno
 */

/**
 * @brief Dual Processor Platform rev 2
 */

#ifndef DPP_BOARD_H_
#define DPP_BOARD_H_


/*
 * --- GPIO mapping ---
 */

#define LED_0                       PORT3, PIN0
#define LED_STATUS                  LED_0
#define LED_ERROR                   LED_0
#define COM_GPIO1                   PORT3, PIN1
#define COM_GPIO2                   PORT3, PIN2
#define COM_PROG2                   PORT3, PIN3
#define COM_GPIO3                   COM_PROG2

/* LED ports */
#define LEDS_PxDIR                  P3DIR
#define LEDS_PxOUT                  P3OUT
#define LEDS_CONF_RED               0x0       /* does not exist */
#define LEDS_CONF_GREEN             PIN0
#define LEDS_CONF_YELLOW            0x0       /* does not exist */

#if BOLT_CONF_ON
  #define BOLT_CONF_SPI             SPI_1
  #define BOLT_CONF_IND_PIN         PORT2, PIN2
  #define BOLT_CONF_MODE_PIN        PORT2, PIN3
  #define BOLT_CONF_REQ_PIN         PORT2, PIN4
  #define BOLT_CONF_ACK_PIN         PORT2, PIN5
  /* IND pin for the outgoing queue (sent messages) */
  #define BOLT_CONF_IND_OUT_PIN     PORT1, PIN1
  #define BOLT_CONF_TIMEREQ_PIN     PORT2, PIN1
  #define BOLT_CONF_FUTUREUSE_PIN   PORT2, PIN6
  #if BOLT_CONF_TIMEREQ_HF_MODE
  /* Note: HF mode cannot be used if LWB_CONF_USE_LF_FOR_WAKEUP is enabled! */
    #define BOLT_CONF_TIMEREQ_TIMERID RTIMER_EXT_HF_0
    #define BOLT_CONF_TIMEREQ_DMATRG  DMA_TRGSRC_TA0CCR0
    #define BOLT_CONF_TIMEREQ_PINMAP  PM_TA0CCR0A
    #define BOLT_CONF_TIMEREQ_CCR     TA0CCR0
  #else /* BOLT_CONF_TIMEREQ_HF_MODE */
    #define BOLT_CONF_TIMEREQ_TIMERID RTIMER_EXT_LF_0
    #define BOLT_CONF_TIMEREQ_DMATRG  DMA_TRCSRC_TA1CCR0
    #define BOLT_CONF_TIMEREQ_PINMAP  PM_TA1CCR0A
    #define BOLT_CONF_TIMEREQ_CCR     TA1CCR0
  #endif /* BOLT_CONF_TIMEREQ_HF_MODE */
#endif /* BOLT_CONF_ON */

/* specify what needs to be done every time before UART is enabled */
#define UART_BEFORE_ENABLE          /* nothing to be done */

/* specify what needs to be done every time before SPI is enabled */
#define SPI_BEFORE_ENABLE(spi)      /* nothing to be done */


#endif /* DPP_BOARD_H_ */

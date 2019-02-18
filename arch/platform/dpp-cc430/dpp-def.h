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
 * @brief platform includes and definitions for the Dual Processor Platform
 */

#ifndef DPP_DEF_H_
#define DPP_DEF_H_

/*
 * --- standard libraries ---
 */

//#include <stdio.h>
//#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <isr_compat.h>


/*
 * --- MCU and compiler definitions ---
 */

#include <msp430.h>

#ifdef __CC430F5147__
#define MCU_DESC                    "CC430F5147"
#define MCU_DEVICE_ID               0x8138    /* TI device ID for this MCU */
#elif defined __CC430F5137__
#define MCU_DESC                    "CC430F5137"
#define MCU_DEVICE_ID               0x3751    /* TI device ID for this MCU */
#endif

#define DPP_PLATFORM_STR            "DPP2"

#define TI_DEVICE_ID                REGVAL16(0x1A04)  /* read ID from TLV */
#define TI_DEVICE_HWREV             REGVAL8(0x01A06)
/* create unique ID from wafer ID and die position (x, y) */
#define TI_UNIQUE_ID                ((uint64_t)REGVAL32(0x01A0A) << 32 | \
                                               REGVAL32(0x01A0E))

/* compiler info */
#if defined(__GNUC__)
#define COMPILER_DESC               "GCC"
#define COMPILER_VERSION            (__GNUC__ * 10000 + \
                                     __GNUC_MINOR__ * 100 + \
                                     __GNUC_PATCHLEVEL__)
#define COMPILER_VERSION_ENC        ((__GNUC__ << 10) + \
                                     (__GNUC_MINOR__ << 5) + \
                                     (__GNUC_PATCHLEVEL__))
#define COMPILER_VERSION_32         (((uint32_t)__GNUC__ * 1000000) + \
                                     ((uint32_t)__GNUC_MINOR__ * 1000) + \
                                     (__GNUC_PATCHLEVEL__))
#define COMPILER_INFO               __VERSION__   /* info string */
#elif defined(__IAR_SYSTEMS_ICC__)
#define COMPILER_DESC               "IAR"
#define COMPILER_VERSION            __VER__
#elif defined(__TI_COMPILER_VERSION__)
#define COMPILER_DESC               "TI"
#define COMPILER_VERSION            __TI_COMPILER_VERSION__
#endif

#define SRAM_START                  0x1c00
#define SRAM_SIZE                   4096          /* starting at 0x1C00 */

#define STACK_CONF_ORIGIN           ((void *)(SRAM_START + SRAM_SIZE))


/*
 * --- defines for CONTIKI ---
 */

/* Contiki requires the definition of the following data types: */
typedef uint32_t clock_time_t;
typedef uint16_t rtimer_clock_t;
#define RTIMER_CLOCK_DIFF(a, b)     ((int64_t)((a) - (b)))

#define RTIMER_CONF_SECOND          ACLK_SPEED


/*
 * --- default settings ---
 */

#ifndef WATCHDOG_CONF_ON
#define WATCHDOG_CONF_ON            0
#endif /* WATCHDOG_CONF_ON */

#ifndef LED_CONF_ON
#define LED_CONF_ON                 1
#define LED_CONF_INVERT             0
#endif /* LED_CONF_ON */

#ifndef BOLT_CONF_ON
#define BOLT_CONF_ON                0
#endif /* BOLT_CONF_ON */

#ifndef SVS_CONF_ON
#define SVS_CONF_ON                 0
#endif /* SVS_CONF_ON */

#ifndef RF_CONF_ON
#define RF_CONF_ON                  1
#endif /* RF_CONF_ON */


/*
 * --- clock configuration ---
 */

/* FLL is only required if one of the following clock sources is used: DCOCLK,
 DCOCLKDIV, FLLREFCLK */
#ifndef CLOCK_CONF_FLL_ON
#define CLOCK_CONF_FLL_ON           1
#endif /* CLOCK_CONF_FLL_ON */

#ifndef CLOCK_CONF_XT1_ON
#define CLOCK_CONF_XT1_ON           1
#endif /* CLOCK_CONF_XT1_ON */

#ifndef CLOCK_CONF_XT2_ON
#define CLOCK_CONF_XT2_ON           RF_CONF_ON
#endif /* CLOCK_CONF_XT2_ON */

#ifndef CLOCK_CONF_XT1_CAP
#define CLOCK_CONF_XT1_CAP          XCAP_0
#endif /* CLOCK_CONF_XT1_CAP */

/* if clock requests for peripherals are disabled, the SMCLK will be gated
 * (forced off) in LPM2/3/4 */
#ifndef CLOCK_CONF_SMCLKREQ_ON
#define CLOCK_CONF_SMCLKREQ_ON      1
#endif /* CLOCK_CONF_SMCLKREQ_ON */

#ifndef CLOCK_CONF_ACLKREQ_ON
#define CLOCK_CONF_ACLKREQ_ON       1
#endif /* CLOCK_CONF_SMCLKREQ_ON */

/* speed of XT1 (low-frequency crystal) */
#define XT1CLK_SPEED                32768UL

/* speed of XT2 (high-frequency crystal) */
#define XT2CLK_SPEED                26000000UL

/* speed of the internal DCO */
#define DCOCLK_SPEED                13000000UL

#if CLOCK_CONF_XT2_ON
  /* if XT2 is not used, then source MCLK and SMCLK from DCOCLK */

  /* source and speed of the Master Clock MCLK */
  #define SELM                      SELM__XT2CLK
  #define DIVM                      DIVM__2
  #define MCLK_SPEED                (XT2CLK_SPEED / 2)      /* 13 MHz */

  /* source and speed of the Sub-System Master Clock SMCLK */
  #define SELS                      SELS__XT2CLK
  #define DIVS                      DIVS__8
  #define SMCLK_SPEED               (XT2CLK_SPEED / 8)      /* 3.25 MHz */

#else /* CLOCK_CONF_XT2_ON */

  /* source and speed of the Master Clock MCLK */
  #define SELM                      SELM__DCOCLKDIV
  #define DIVM                      DIVM__1
  #define MCLK_SPEED                (DCOCLK_SPEED)          /* 13 MHz */

  /* source and speed of the Sub-System Master Clock SMCLK */
  #define SELS                      SELS__DCOCLKDIV
  #define DIVS                      DIVS__4
  #define SMCLK_SPEED               (DCOCLK_SPEED / 4)      /* 3.25 MHz */

#endif /* CLOCK_CONF_XT2_ON */

/* source and speed of the Auxiliary Clock ACLK */
#if CLOCK_CONF_XT1_ON
  #define SELA                      SELA__XT1CLK            /* 32768 Hz */
#else /* CLOCK_CONF_XT1_ON */
  #define SELA                      SELA__REFOCLK           /* ~32.8kHz */
#endif /* CLOCK_CONF_XT1_ON */
#define DIVA                        DIVA__1
#define ACLK_SPEED                  (XT1CLK_SPEED / 1)


/*
 * --- peripherals ---
 */

/* whether or not rtimer-ext should reserve a CCR for the etimer */
#ifndef RTIMER_EXT_CONF_USE_ETIMER
#define RTIMER_EXT_CONF_USE_ETIMER  1
#endif /* RTIMER_EXT_CONF_USE_ETIMER */

#ifndef RTIMER_EXT_CONF_HF_ENABLE
#define RTIMER_EXT_CONF_HF_ENABLE   1
#endif /* RTIMER_EXT_CONF_HF_ENABLE */

/* specify the number of timer modules */
#define RTIMER_EXT_CONF_NUM_HF      (5 - RF_CONF_ON)                  /* number of high-frequency timers */
#define RTIMER_EXT_CONF_NUM_LF      (3 - RTIMER_EXT_CONF_USE_ETIMER)  /* number of low-frequency timers */
/* TA1 offers 3 CCR registers, CCR1 is used for the rtimer module, CCR2 for the etimer */

#define RTIMER_ARCH_TIMER_ID        RTIMER_EXT_LF_1
#define ETIMER_ARCH_TIMER_ID        RTIMER_EXT_LF_2

/* specify the number of SPI modules */
#define SPI_CONF_NUM_MODULES        2


/*
 * --- helper macros ---
 */

#define WAIT_MS(ms)                 __delay_cycles((MCLK_SPEED / 1000) * ms)
#define DELAY(ms)                   __delay_cycles((MCLK_SPEED / 1000) * ms)

/* check whether the high-frequency crystal XT2 is permanently enabled */
#define IS_XT2_ENABLED()            (!(UCSCTL6 & XT2OFF))

/* macros for enabling the external crystal oscillators */
#define ENABLE_XT2()                (UCSCTL6 &= ~XT2OFF)
#define ENABLE_XT1()                (UCSCTL6 &= ~XT1OFF)

/* macros for disabling the external crystal oscillators */
#define DISABLE_XT2()               (UCSCTL6 |= XT2OFF)
#define DISABLE_XT1()               (UCSCTL6 |= XT1OFF)

/* disable automatic clock requests for ACLK */
#define DISABLE_ACLK()              { UCSCTL8 &= ~ACLKREQEN; }

/* disable automatic clock requests for SMCLK */
#define DISABLE_SMCLK()             { UCSCTL8 &= ~SMCLKREQEN; }

/* enable the FLL control loop */
#define ENABLE_FLL()                (__bic_status_register(SCG0))

/* disable the FLL control loop */
#define DISABLE_FLL()               (__bis_status_register(SCG0))

/* wait for the oscillator fault flag to clear */
#define WAIT_FOR_OSC()  do { \
                          UCSCTL7 &= ~(XT1LFOFFG + DCOFFG + XT2OFFG); \
                          SFRIFG1 &= ~OFIFG; \
                        } while (SFRIFG1 & OFIFG)

/*
 * --- LPM ---
 */

/* what to do before entering and after exiting deepsleep mode */
#define AFTER_DEEPSLEEP()       if(UCSCTL6 & XT2OFF) { \
                                  SFRIE1  &= ~OFIE; \
                                  ENABLE_XT2(); \
                                  WAIT_FOR_OSC(); \
                                  UCSCTL4  = SELA | SELS | SELM; \
                                  __delay_cycles(100); /* errata PMM11/12? */\
                                  UCSCTL5  = DIVA | DIVS | DIVM; \
                                  UCSCTL7  = 0; /* errata UCS11 */ \
                                  SFRIE1  |= OFIE; \
                                  TA0CTL  |= MC_2; \
                                  P1SEL    = (BIT2 | BIT3 | BIT4 | \
                                              BIT6); \
                                  P1REN    = 0; /* disable pullup */ \
                                }
/* note: errata PMM11 should not affect this clock config; MCLK is sourced from
 * DCO, but DCO is not running at >4MHz and clock divider is 2 */
/* disable all peripherals, reconfigure the GPIOs and disable XT2 */
#define BEFORE_DEEPSLEEP()      {\
                                  TA0CTL &= ~MC_3; /* stop TA0 */\
                                  P1DIR    = (BIT2 | BIT3 | BIT4 | BIT6); \
                                  P1OUT    = (BIT5 | BIT6); \
                                  P1REN    = BIT5; /* enable pullup */ \
                                  P1SEL = 0; /* reconfigure GPIOs */\
                                  /* set clock source to DCO (3.25MHz) */\
                                  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV | \
                                            SELM__DCOCLKDIV; \
                                  UCSCTL5 |= DIVM__4; /* errata PMM11 */ \
                                  UCSCTL7  = 0; /* errata UCS11 */ \
                                  DISABLE_XT2(); \
                                }


/*
 * --- board specific definitions (pin mappings etc.) ---
 */

#include "dpp-board.h"


/*
 * --- mandatory includes for this platform ---
 */

/* RF config */
#if RF_CONF_ON
#include "../dpp-cc430/rf1a-SmartRF-settings/868MHz-2GFSK-250kbps.h"
#endif /* RF_CONF_ON */

#include "pmm.h"
#define DEBUG_PRINT_CONF_ON_FATAL()     PMM_TRIGGER_POR

#include "uart.h"
#define DEBUG_PRINT_CONF_DISABLE_UART   1
#define DEBUG_PRINT_CONF_UART_ENABLE()  uart_enable(1)
#define DEBUG_PRINT_CONF_UART_DISABLE() uart_enable(0)

#include "gpio.h"
#define DEBUG_PRINT_ERROR_LED_ON        PIN_SET(LED_ERROR)

#include "rtimer-ext.h"

#endif /* DPP_DEF_H_ */

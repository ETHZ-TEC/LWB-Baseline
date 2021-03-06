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

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * application specific config file to override default settings
 */

// #define FLOCKLAB                          /* uncomment to run on FlockLAB */
#define HOST_ID                         1
#define NUM_NODES                       30
#define SOURCE_IPI                      4    /* seconds */

#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"
  /* set the highest antenna gain if the program runs on FlockLAB */
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #ifdef PLATFORM_SKY
    #define GLOSSY_TX_PIN               FLOCKLAB_LED2
    #define GLOSSY_RX_PIN               FLOCKLAB_LED3
  #else  /* PLATFORM_SKY */
    #define RF_GDO2_PIN                 FLOCKLAB_INT1
  #endif /* PLATFORM_SKY */
  #define LWB_CONF_TASK_ACT_PIN         FLOCKLAB_INT2
  #define DEBUG_PRINT_CONF_TASK_ACT_PIN FLOCKLAB_INT2
  #define APP_TASK_ACT_PIN              FLOCKLAB_INT2
#endif /* FLOCKLAB */

/* platform specific config */
#ifdef PLATFORM_SKY
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    #define GLOSSY_RX_PIN               ADC2
    //#define GLOSSY_DEBUG_PIN            ADC0
    //#define LWB_CONF_TASK_ACT_PIN       ADC2
    //#define DEBUG_PRINT_CONF_TASK_ACT_PIN ADC2
    //#define APP_TASK_ACT_PIN            ADC2
  #endif /* FLOCKLAB */
  /* RF */
  #define RF_CHANNEL                    11
  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL
  /* stats */
  #define DCSTAT_CONF_ON                1

#elif defined PLATFORM_DPP_CC430
  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            COM_GPIO1
    #define RF_GDO2_PIN                 COM_GPIO2
    #define LWB_CONF_TASK_ACT_PIN       COM_GPIO3
    #define DEBUG_PRINT_CONF_TASK_ACT_PIN COM_GPIO3
    #define APP_TASK_ACT_PIN            COM_GPIO3
  #endif /* FLOCKLAB */
  /* stats */
  #define DCSTAT_CONF_ON                1

#else
  #error "unknown target platform"
#endif

/* LWB configuration */
#define LWB_SCHED_MIN_ENERGY             /* use the minimum energy scheduler */
#define LWB_CONF_SCHED_PERIOD_IDLE      5        /* define the period length */
#define LWB_CONF_SCHED_PERIOD_MIN       2
#define LWB_CONF_SCHED_PERIOD_MAX       15
#define LWB_CONF_OUT_BUFFER_SIZE        4
#define LWB_CONF_IN_BUFFER_SIZE         NUM_NODES
#define LWB_CONF_MAX_PKT_LEN            80
#define LWB_CONF_MAX_DATA_PKT_LEN       15
#define LWB_CONF_MAX_DATA_SLOTS         NUM_NODES

#define LWB_CONF_T_SCHED                (LWB_RTIMER_SECOND / 40)     /* 25ms */
#define LWB_CONF_T_DATA                 (LWB_RTIMER_SECOND / 50)     /* 20ms */
#define LWB_CONF_T_GUARD                (LWB_RTIMER_SECOND / 2000)  /* 0.5ms */
#define LWB_CONF_T_GUARD_1              (LWB_RTIMER_SECOND / 2000)  /* 0.5ms */
#define LWB_CONF_T_GUARD_2              (LWB_RTIMER_SECOND / 1000)    /* 1ms */
#define LWB_CONF_T_GUARD_3              (LWB_RTIMER_SECOND / 500)     /* 2ms */
#define LWB_CONF_T_GAP                  (LWB_RTIMER_SECOND / 200)     /* 5ms */
#define LWB_CONF_T_CONT                 (LWB_RTIMER_SECOND / 125)     /* 8ms */
#define LWB_CONF_TX_CNT_SCHED           3
#define LWB_CONF_TX_CNT_DATA            3
#define LWB_CONF_T_SCHED2_START         (LWB_RTIMER_SECOND)            /* 1s */
#define LWB_CONF_T_SILENT               0            /* disable this feature */

/* Debug */
#define DEBUG_PRINT_CONF_STACK_GUARD    (SRAM_START + SRAM_SIZE - 0x0200)


#endif /* PROJECT_CONF_H_ */

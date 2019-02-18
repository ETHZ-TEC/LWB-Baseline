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
            Fabian Mager
 */

/**
 * @brief Low-Power Wireless Bus Test Application
 */

#include "contiki.h"
#include "dc-stat.h"
#include "debug-print.h"
#include "glossy.h"
#include "gpio.h"
#include "lwb.h"
#include "node-id.h"

#include "isr_compat.h"
#include "my2c.h"
#include "project-conf.h"
#include "stdint.h"
#include "testbed.h"
#include "watchdog.h"

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */

#ifdef APP_DEBUG_PIN
#define APP_DEBUG_ON PIN_SET(APP_DEBUG_PIN)
#define APP_DEBUG_OFF PIN_CLR(APP_DEBUG_PIN)
#define APP_DEBUG_TOGGLE PIN_XOR(APP_DEBUG_PIN)
#else
#define APP_DEBUG_ON
#define APP_DEBUG_OFF
#define APP_DEBUG_TOGGLE
#endif /* APP_DEBUG_PIN */
/*---------------------------------------------------------------------------*/

typedef enum { ROLE_NOTHING = 0, ROLE_SOURCE = 1, ROLE_DESTINATION = 2 } role_t;

/*---------------------------------------------------------------------------*/

volatile config_t __attribute((section(".testbedConfigSection"))) cfg;
static process_event_t FALLING_EDGE_EV;
static role_t role = ROLE_NOTHING;
static pattern_t comPattern;
static uint8_t pkt_buffer[LWB_CONF_MAX_DATA_PKT_LEN];
static uint8_t stream_state = 0;

/*---------------------------------------------------------------------------*/

PROCESS(read_message, "Read message process");
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process, &read_message);

/*---------------------------------------------------------------------------*/

static void init_pins(void) {
	// init debug pins
#ifdef GLOSSY_START_PIN
	PIN_CLR(GLOSSY_START_PIN);
	PIN_CFG_OUT(GLOSSY_START_PIN);
#endif /* GLOSSY_START_PIN */
#ifdef GLOSSY_TX_PIN
	PIN_CLR(GLOSSY_TX_PIN);
	PIN_CFG_OUT(GLOSSY_TX_PIN);
#endif /* GLOSSY_TX_PIN */
#ifdef GLOSSY_RX_PIN
	PIN_CLR(GLOSSY_RX_PIN);
	PIN_CFG_OUT(GLOSSY_RX_PIN);
#endif /* GLOSSY_RX_PIN */
#ifdef GLOSSY_DEBUG_PIN
	PIN_CLR(GLOSSY_DEBUG_PIN);
	PIN_CFG_OUT(GLOSSY_DEBUG_PIN);
#endif /* GLOSSY_DEBUG_PIN */
#ifdef DEBUG_PRINT_CONF_TASK_ACT_PIN
	PIN_CLR(DEBUG_PRINT_CONF_TASK_ACT_PIN);
	PIN_CFG_OUT(DEBUG_PRINT_CONF_TASK_ACT_PIN);
#endif /* DEBUG_PRINT_CONF_TASK_ACT_PIN */
#ifdef APP_TASK_ACT_PIN
	PIN_CLR(APP_TASK_ACT_PIN);
	PIN_CFG_OUT(APP_TASK_ACT_PIN);
#endif /* APP_TASK_ACT_PIN */
#ifdef APP_DEBUG_PIN
	PIN_CLR(APP_DEBUG_PIN);
	PIN_CFG_OUT(APP_DEBUG_PIN);
#endif /* APP_DEBUG_PIN */
#ifdef DCSTAT_CPU_ON_PIN
	PIN_CLR(DCSTAT_CPU_ON_PIN);
	PIN_CFG_OUT(DCSTAT_CPU_ON_PIN);
#endif /* DCSTAT_CPU_ON_PIN */
#ifdef GERNERAL_DEBUG_PIN
	PIN_CLR(GERNERAL_DEBUG_PIN);
	PIN_CFG_OUT(GERNERAL_DEBUG_PIN);
#endif /* GERNERAL_DEBUG_PIN */
#ifdef LWB_RND_PIN
	PIN_CLR(LWB_RND_PIN);
	PIN_CFG_OUT(LWB_RND_PIN);
#endif /* LWB_RND_PIN */

	// configure pin connected to EEPROM
	if (role == ROLE_SOURCE) {
		// configure pin to input with interrupt
		P2DIR &= ~BV(EVENT_PIN);
		P2SEL &= ~BV(EVENT_PIN);
		P2IES |= BV(EVENT_PIN);
		P2IFG &= ~BV(EVENT_PIN);

		// wait until the pin is initially settled
		clock_delay(10000);
		while ((P2IN & BIT6) != 0)
			;
	}

	if (role == ROLE_DESTINATION) {
		// configure pin to output
		P2SEL &= ~BV(EVENT_PIN);
		P2DIR |= BV(EVENT_PIN);
		P2OUT &= ~BV(EVENT_PIN);
	}
}

/*---------------------------------------------------------------------------*/

// Determine the role of this node based on the data in cfg.
static void determine_role(void) {
	unsigned int i = 0;
	unsigned int j = 0;
	for (i = 0; i < TB_NUMPATTERN; i++) {
		// Skip unused patterns.
		if (cfg.p[i].traffic_pattern == 0) continue;

		for (j = 0; j < TB_NUMNODES; j++) {
			if (node_id == cfg.p[i].source_id[j]) {
				role = ROLE_SOURCE;
				comPattern = cfg.p[i];
				return;
			}
			if (node_id == cfg.p[i].destination_id[j]) {
				role = ROLE_DESTINATION;
				comPattern = cfg.p[i];
				return;
			}
		}
	}
}

/*---------------------------------------------------------------------------*/

static void activate_stream(void) {
	if (stream_state != LWB_STREAM_STATE_ACTIVE) {
		stream_state = lwb_stream_get_state(1);
		if (stream_state == LWB_STREAM_STATE_INACTIVE) {
			lwb_stream_req_t my_stream;
			/* request a stream */
			if (comPattern.periodicity == 0) { // aperiodic
				my_stream = (lwb_stream_req_t){node_id, 0, 1,
				                               (uint16_t)(comPattern.aperiodic_lower_bound / 1000)};
			}
			else {
				my_stream =
				    (lwb_stream_req_t){node_id, 0, 1, (uint16_t)(comPattern.periodicity / 1000)};
			}
			if (!lwb_request_stream(&my_stream, 0)) { DEBUG_PRINT_ERROR("stream request failed"); }
		}
	}
}

/*---------------------------------------------------------------------------*/

ISR(PORT2, __eeprom_isr) {
	P2IFG = 0;
	// A destination node writes the received data into the EEPROM and toggles the GPIO line to the
	// observer. This however triggers the ISR on the destination itself which would result in
	// erroneous read_message calls. This is possible since we enable and disable interrupts for
	// this port in the LWB code for all nodes but the host (which is always a destination).
	// However, in the LWB code we have no means to distinct between source and destination nodes
	// as we do here in the app code, so we have to insert this check.
	if (role == ROLE_SOURCE) { process_poll(&read_message); }
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(read_message, ev, data) {
	PROCESS_BEGIN();

	while (1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		// i2c start
		my2c_start();
		// write address on the bus
		my2c_write(0x50 << 1);
		// write memory address (2 bytes)
		my2c_write(comPattern.msg_offsetH);
		my2c_write(comPattern.msg_offsetL);
		// disable the bus and wait a bit
		my2c_stop();
		clock_delay(100);
		my2c_start();
		// write address on the bus and set read bit
		my2c_write((0x50 << 1) | 1);
		// read back all the data, on the last byte indicate end
		int i;
		for (i = 0; i < comPattern.msg_length; i++) {
			if (i == (comPattern.msg_length - 1))
				pkt_buffer[i] = my2c_read(0);
			else
				pkt_buffer[i] = my2c_read(1);
		}
		// i2c stop
		my2c_stop();

		lwb_send_pkt(LWB_RECIPIENT_BROADCAST, 1, pkt_buffer, comPattern.msg_length);
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/

static void deliver_data(uint8_t *data, uint16_t len) {
	rtimer_ext_clock_t lastTime = LWB_RTIMER_NOW();
	while ((LWB_RTIMER_NOW() - lastTime) < 660)
		;

	// raise gpio pin to indicate write operation
	P2OUT |= BV(EVENT_PIN);

	// start i2c communication
	my2c_start();
	// write address on the bus
	my2c_write(0x50 << 1);
	// write memory address (2 bytes)
	my2c_write(comPattern.msg_offsetH);
	my2c_write(comPattern.msg_offsetL);

	// write messages up to the length in the struct
	int i;
	for (i = 0; i < len; i++) { my2c_write(data[i]); }
	// stop i2c communication
	my2c_stop();

	// After STOP command, EEPROM executes write cycle for at most 5ms. (CAT24M01-D p.3)
	rtimer_ext_clock_t current = LWB_RTIMER_NOW();
	while ((LWB_RTIMER_NOW() - current) < 165) // ~164 cycles (32kHz) are 5ms
		;
	// lower gpio pin to indicate finished write
	P2OUT &= ~BV(EVENT_PIN);
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(app_process, ev, data) {
	PROCESS_BEGIN();

	determine_role();
	init_pins();

	// allocate events for rising and falling edge
	FALLING_EDGE_EV = process_alloc_event();
	// the software i2c may be stuck for a long time
	watchdog_stop();
	// enable and stop the i2c (otherwise it would block the bus)
	my2c_enable();
	my2c_stop();

	/* start the LWB thread */
	lwb_start(0, &app_process, node_id == HOST_ID);

	/* main loop of this application task */
	while (1) {
		/* the app task should not do anything until it is explicitly granted
		 * permission (by receiving a poll event) by the LWB task */
		TASK_SUSPENDED;
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
		TASK_ACTIVE; /* application task runs now */

		if (role == ROLE_SOURCE) {
			activate_stream();
		}
		else if (role == ROLE_DESTINATION) {
			uint16_t cnt = 0;
			uint16_t pkt_len;
			uint16_t sender_id;
			do {
				cnt++;
				pkt_len = lwb_rcv_pkt(pkt_buffer, &sender_id, 0);
				if (pkt_len) {
					// Check if there is only a single source.
					if (comPattern.traffic_pattern <= 2) {
						if (sender_id == comPattern.source_id[0]) {
							deliver_data(pkt_buffer, pkt_len);
						}
					}
					else {
						deliver_data(pkt_buffer, pkt_len);
					}
				}
			} while (pkt_len);
		}

		// Empty receive queue with all nodes to avoid overflows at source nodes and relays.
		lwb_reset_rcv_queue();
		/* let the debug print process run */
		debug_print_poll();
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/

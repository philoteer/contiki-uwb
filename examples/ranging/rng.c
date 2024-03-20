/*
 * Copyright (c) 2017, University of Trento.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *	copyright notice, this list of conditions and the following
 *	disclaimer in the documentation and/or other materials provided
 *	with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *	products derived from this software without specific prior
 *	written permission.
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

#include <stdio.h>

#include "contiki.h"
#include "core/net/linkaddr.h"
#include "dw1000-cir.h"
#include "dw1000-ranging.h"
#include "dw1000.h"
#include "leds.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/rime/rime.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
/*---------------------------------------------------------------------------*/
#define RANGING_TIMEOUT (CLOCK_SECOND / 10)
/*---------------------------------------------------------------------------*/
#if LINKADDR_SIZE == 2
linkaddr_t dst = {{0x49, 0xD4}};
#elif LINKADDR_SIZE == 8
linkaddr_t dst = {{0x01, 0x3a, 0x61, 0x02, 0xc4, 0x40, 0x5a, 0x34}};
#endif
/*---------------------------------------------------------------------------*/

/* Option to read and print CIR */
#define ACQUIRE_CIR 1  // 1 = enable CIR acquisition
/* Acquire CIR samples around the first path index */
#define CIR_IDX_MODE DW1000_CIR_IDX_RELATIVE
#define CIR_IDX_START (-16)
#define CIR_MAX_SAMPLES 128  // number of CIR samples to acquire

#if ACQUIRE_CIR
#define MAX_PRINTING_TIME \
	(CLOCK_SECOND / 20)  // estimated time needed to print a full CIR (depends
						 // on the platform)
#define CIR_DUMP_DELAY \
	(CLOCK_SECOND / 200)  // a delay inserted after printing CIR to let the USB
						  // buffer get emptied
#else
#define MAX_PRINTING_TIME \
	(CLOCK_SECOND / 1000)  // time needed to print ranging results and
						   // diagnostics (USB or RTT output)
// #define MAX_PRINTING_TIME (CLOCK_SECOND / 100)	// choose this instead
// if using serial output
#define CIR_DUMP_DELAY 0
#endif

#if ACQUIRE_CIR
dw1000_cir_sample_t cir_buf[CIR_MAX_SAMPLES + 1];  // +1 is required!
#endif

PROCESS_THREAD(range_process, ev, data) {
	int seqn = 0;
	static struct etimer et;
	static struct etimer timeout;
	static int status;

	PROCESS_BEGIN();

	printf("I am %02x%02x dst %02x%02x\n", linkaddr_node_addr.u8[0],
		   linkaddr_node_addr.u8[1], dst.u8[0], dst.u8[1]);

	if (!linkaddr_cmp(&linkaddr_node_addr, &dst)) {
		etimer_set(&et, 5 * CLOCK_SECOND);
		PROCESS_WAIT_UNTIL(etimer_expired(&et));
		printf("I am %02x%02x ranging with %02x%02x\n",
			   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], dst.u8[0],
			   dst.u8[1]);

		while (1) {
			printf("R req\n");

#if ACQUIRE_CIR
			dw1000_ranging_acquire_diagnostics(CIR_IDX_MODE, CIR_IDX_START,
											   CIR_MAX_SAMPLES, cir_buf);
#else  // only request diagnostics
			dw1000_ranging_acquire_diagnostics(0, 0, 0, NULL);
#endif

			status = range_with(&dst, DW1000_RNG_DS);
			if (!status) {
				printf("R req failed\n");
			} else {
				etimer_set(&timeout, RANGING_TIMEOUT);
				PROCESS_YIELD_UNTIL(
					(ev == ranging_event || etimer_expired(&timeout)));
				if (etimer_expired(&timeout)) {
					printf("R TIMEOUT\n");
				} else if (((ranging_data_t *)data)->status) {
					seqn++;
					ranging_data_t *d = data;
					printf("R success: %d bias %d\n",
						   (int)(100 * d->raw_distance),
						   (int)(100 * d->distance));

#if ACQUIRE_CIR
					uint16_t cir_start = cir_buf[0].u32;
					uint16_t cir_fp_int = d->rxdiag.firstPath >> 6;
					uint16_t cir_fp_frac = d->rxdiag.firstPath & 0x3f;
					// fp (first path) is printed as the integer part and the
					// 1/64 fractional part "fp a#b" means (a + b/64)
					printf("CIR [%lu] %02x%02x->%02x%02x %d#%d [%d:%d] ", seqn,
						   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
						   dst.u8[0], dst.u8[1], cir_fp_int, cir_fp_frac,
						   cir_start, d->cir_samples_acquired);
					dw1000_print_cir_hex(cir_buf + 1, d->cir_samples_acquired);
#endif

				} else {
					printf("R FAIL\n");
				}
			}
			etimer_set(&et, CLOCK_SECOND / 50);
			PROCESS_WAIT_UNTIL(etimer_expired(&et));
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*
 * Raw samples of tc32_rng's two credited sources, for assessing their
 * min-entropy on the chip (TLSR_RNG_MEASURE, measurement images only):
 * the update receiver's command 0xff0e (src/tlsr_usb_ota.c), read with
 * telink_ota.py rng-samples.
 *
 * One buffer of TLSR_RNG_MEASURE_BYTES octets holds one of:
 * - the restart batch, taken at boot right after tc32_rng's init: the
 *   first RESTART_JITTER jitter samples, then (from octet RESTART_JITTER)
 *   the first RESTART_ADC ADC codes, each source's samples one unbroken run;
 * - a take: one source's samples, as many as the buffer holds, in one run.
 * A jitter sample is one octet, bits 10:3 of the system timer at a change
 * of the 32 kHz count (tc32_rng_jitter_samples()); an ADC code is two
 * octets, little-endian, the low 16 bits of the DMA's word for a
 * conversion of VBAT (tc32_rng_adc_samples()). The generator absorbs both
 * as it always does. The buffer is cleared before each fill, and the
 * counts below are the samples the hooks copied: fewer than asked when the
 * 32 kHz count or the ADC stopped, never anything left from before.
 *
 * Request byte 11: 0 status, 1 take (byte 12: 0 jitter, 1 ADC), 2 read
 * (bytes 12-13: the page, little-endian). The status and the take answer:
 *   11     what the buffer holds: 0 nothing, 1 the restart batch,
 *          2 a jitter take, 3 an ADC take
 *   12-13  its octets
 *   14-15  samples of the first source (jitter in the restart batch)
 *   16-17  samples of the second (the restart batch's ADC codes; 0 else)
 *   18-19  the buffer's size in octets
 *   20     times page 0 of the restart batch was read in this boot
 *   21-22  "ZC"
 *   23-26  the uptime in ms when the buffer's samples were taken
 *   27-28  takes since boot
 * The read answers bytes 11-28 with 18 octets of the buffer from page * 18
 * (zeros past the content).
 * Request byte 11 = 3, readiness: tc32_rng_collect() as the BLE stack calls
 * it before advertising (at once when the generator is ready), then
 * tc32_rng_get_readiness(); the answer:
 *   11     the collect's return value (0 ready, else -errno), as an int8
 *   12     the state before the collect, 13 after (0 not ready, 1 ready,
 *          2 the known-answer test failed)
 *   14     the credited bits (up to 128)
 *   15     1 if the seed record counted at init
 *   16-17  the collect's ms
 *   18-19  ADC windows credited since boot
 *   21-22  "ZC"
 *   23-24  ADC windows refused for a code out of VBAT's range
 *   25-26  ADC windows refused by the health tests
 * Status 8 for anything else.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/byteorder.h>

#include "tlsr_rng_measure.h"

#define BYTES          CONFIG_TLSR_RNG_MEASURE_BYTES
#define RESTART_JITTER 1000U
#define RESTART_ADC    1001U
#define PAGE           18U

BUILD_ASSERT(RESTART_JITTER + 2U * RESTART_ADC <= BYTES, "the restart batch fits the buffer");

enum {
	NONE,
	RESTART,
	JITTER,
	ADC,
};

static uint8_t buf[BYTES] __aligned(2);
static uint16_t used, first, second, takes;
static uint8_t kind, restart_reads;
static uint32_t taken_ms;

/* The restart batch: the jitter's first samples, then the ADC's. */
static int restart_batch(void)
{
	memset(buf, 0, sizeof(buf));
	taken_ms = k_uptime_get_32();
	first = (uint16_t)tc32_rng_jitter_samples(buf, RESTART_JITTER - 1U);
	second = (uint16_t)tc32_rng_adc_samples((uint16_t *)&buf[RESTART_JITTER], RESTART_ADC);
	used = RESTART_JITTER + 2U * second;
	kind = RESTART;
	return 0;
}

SYS_INIT(restart_batch, APPLICATION, 0);

static void take(uint8_t source)
{
	memset(buf, 0, sizeof(buf));
	taken_ms = k_uptime_get_32();
	second = 0U;
	if (source == 0U) {
		first = (uint16_t)tc32_rng_jitter_samples(buf, BYTES - 1U);
		used = first;
		kind = JITTER;
	} else {
		first = (uint16_t)tc32_rng_adc_samples((uint16_t *)buf, BYTES / 2U);
		used = 2U * first;
		kind = ADC;
	}
	takes++;
}

static void status(uint8_t r[33])
{
	r[11] = kind;
	sys_put_le16(used, &r[12]);
	sys_put_le16(first, &r[14]);
	sys_put_le16(second, &r[16]);
	sys_put_le16(BYTES, &r[18]);
	r[20] = restart_reads;
	r[21] = 'Z';
	r[22] = 'C';
	sys_put_le32(taken_ms, &r[23]);
	sys_put_le16(takes, &r[27]);
}

static void readiness(uint8_t r[33])
{
	struct tc32_rng_readiness a;
	uint32_t t0;
	uint8_t before;
	int err;

	tc32_rng_get_readiness(&a);
	before = a.state;
	t0 = k_uptime_get_32();
	err = tc32_rng_collect();
	t0 = k_uptime_get_32() - t0;
	tc32_rng_get_readiness(&a);
	memset(&r[11], 0, PAGE);
	r[11] = (uint8_t)(int8_t)err;
	r[12] = before;
	r[13] = a.state;
	r[14] = a.credit_bits;
	r[15] = a.record;
	sys_put_le16((uint16_t)MIN(t0, UINT16_MAX), &r[16]);
	sys_put_le16(a.adc_windows, &r[18]);
	r[21] = 'Z';
	r[22] = 'C';
	sys_put_le16(a.adc_out_of_range, &r[23]);
	sys_put_le16(a.adc_health, &r[25]);
}

int tlsr_rng_measure(uint8_t r[33])
{
	uint8_t op = r[11];

	if (op == 0U) {
		status(r);
	} else if (op == 1U && r[12] <= 1U) {
		take(r[12]);
		status(r);
	} else if (op == 3U) {
		readiness(r);
	} else if (op == 2U) {
		uint32_t at = (uint32_t)sys_get_le16(&r[12]) * PAGE;

		if (at == 0U && kind == RESTART && restart_reads < UINT8_MAX) {
			restart_reads++;
		}
		memset(&r[11], 0, PAGE);
		if (at < used) {
			memcpy(&r[11], &buf[at], MIN(PAGE, used - at));
		}
	} else {
		return -EINVAL;
	}
	return 0;
}

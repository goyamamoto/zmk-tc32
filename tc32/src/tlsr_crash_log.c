/*
 * What ran before a reset the firmware did not ask for (CONFIG_TLSR_CRASH_LOG):
 * a record in RAM that a watchdog or software reset keeps (__noinit; a
 * power-on fills the RAM with noise, which the magic and the checksum reject).
 *
 * - Every 100 ms the system timer's interrupt notes the uptime, the thread it
 *   interrupted and the interrupted PC (the return address the interrupt
 *   entry pushes first, at the top of the interrupt stack), with the watchdog
 *   timer's count (how long since it was last fed), in a ring of 16.
 * - A fatal error notes its reason, the thread and the exception frame's PC
 *   and LR, then halts as before (the watchdog resets the chip 4 s later).
 * - At boot the record of the boot before is moved to a copy that the USB
 *   update interface's command 0xff0d reads (telink_ota.py crash), and the
 *   live record starts again.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/fatal.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/sys_io.h>

#define CRASH_MAGIC 0x43524c47U
#define SAMPLES     16U
#define SAMPLE_MS   100

struct crash_sample {
	uint32_t ms;
	uint32_t thread;
	uint32_t pc;
	uint32_t wd;     /* the watchdog timer's count (timer 2, the 48 MHz system clock): counts since the last feed */
};

struct crash_log {
	uint32_t magic;
	uint32_t next;          /* the next sample's slot (counts on) */
	struct crash_sample s[SAMPLES];
	uint32_t fatal_reason;  /* 0xffffffff: none */
	uint32_t fatal_thread;
	uint32_t fatal_pc;
	uint32_t fatal_lr;
	uint32_t fatal_ms;
	uint32_t crc;           /* CRC-32 of everything before it */
};

static __noinit struct crash_log live;
static struct crash_log prev; /* the boot before's, if its record checked */

extern char z_interrupt_stacks[];
extern char __z_interrupt_stack_SIZEOF[];

static void seal(void)
{
	live.crc = crc32_ieee((const uint8_t *)&live, offsetof(struct crash_log, crc));
}

static void sample(struct k_timer *t)
{
	ARG_UNUSED(t);
	uint32_t top = (uint32_t)(uintptr_t)z_interrupt_stacks + (uint32_t)(uintptr_t)__z_interrupt_stack_SIZEOF;
	struct crash_sample *s = &live.s[live.next % SAMPLES];

	s->ms = k_uptime_get_32();
	s->thread = (uint32_t)(uintptr_t)k_current_get();
	s->pc = *(volatile uint32_t *)(uintptr_t)(top - 4U);
	s->wd = sys_read32(0x800638U);
	live.next++;
	seal();
}

static K_TIMER_DEFINE(sample_timer, sample, NULL);

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	live.fatal_reason = reason;
	live.fatal_thread = (uint32_t)(uintptr_t)k_current_get();
	live.fatal_pc = esf != NULL ? esf->pc : 0U;
	live.fatal_lr = esf != NULL ? esf->lr : 0U;
	live.fatal_ms = k_uptime_get_32();
	seal();
	k_fatal_halt(reason);
}

size_t tlsr_crash_log_read(size_t at, uint8_t *out, size_t n)
{
	if (at >= sizeof(prev)) {
		return 0;
	}
	n = MIN(n, sizeof(prev) - at);
	memcpy(out, (const uint8_t *)&prev + at, n);
	return n;
}

static int crash_log_init(void)
{
	if (live.magic == CRASH_MAGIC &&
	    live.crc == crc32_ieee((const uint8_t *)&live, offsetof(struct crash_log, crc))) {
		prev = live;
	}
	memset(&live, 0, sizeof(live));
	live.magic = CRASH_MAGIC;
	live.fatal_reason = 0xffffffffU;
	seal();
	k_timer_start(&sample_timer, K_MSEC(SAMPLE_MS), K_MSEC(SAMPLE_MS));
	return 0;
}

SYS_INIT(crash_log_init, APPLICATION, 0);

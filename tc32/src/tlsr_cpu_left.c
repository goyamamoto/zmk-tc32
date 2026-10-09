/*
 * The CPU the lowest-priority thread gets (tlsr_cpu_left.h).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "tlsr_cpu_left.h"


static uint32_t count_until(uint32_t cycles)
{
	uint32_t t0 = k_cycle_get_32();
	uint32_t n = 0U;

	while (k_cycle_get_32() - t0 < cycles) {
		n++;
	}
	return n;
}

/*
 * The full rate: the loop's count for 1 ms with interrupts off (after 0.1 ms
 * of it to fill the cache), taken once at boot, before the links run, so
 * that no measurement later holds interrupts (a connection event's timer,
 * the USB poll and the scan timer would wait those 1.1 ms).
 */
static uint32_t full_per_ms;

static int calibrate(void)
{
#ifndef CONFIG_ARCH_POSIX
	const uint32_t per_ms = sys_clock_hw_cycles_per_sec() / 1000U;
	unsigned int key = irq_lock();

	(void)count_until(per_ms / 10U);
	full_per_ms = count_until(per_ms);
	irq_unlock(key);
#endif
	return 0;
}

SYS_INIT(calibrate, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * The CPU the lowest-priority thread gets: this thread drops to that priority
 * and counts the loop for TLSR_CPU_LEFT_MS, interrupts on, against the full
 * rate; their ratio is the milliseconds of CPU it got, so the percent over
 * 100 ms. The emulator's fixed cost per instruction shows nothing of the
 * instruction cache's misses on the chip; this is measured there.
 */
uint8_t tlsr_cpu_left_percent(void)
{
#ifdef CONFIG_ARCH_POSIX
	return 0xffU; /* native_sim's clock does not advance in a busy loop */
#else
	const uint32_t per_ms = sys_clock_hw_cycles_per_sec() / 1000U;
	k_tid_t self = k_current_get();
	int prio = k_thread_priority_get(self);

	if (full_per_ms == 0U) {
		return 0xffU;
	}
	k_thread_priority_set(self, K_LOWEST_APPLICATION_THREAD_PRIO);
	uint32_t got = count_until(TLSR_CPU_LEFT_MS * per_ms);

	k_thread_priority_set(self, prio);
	return (uint8_t)MIN(got * (100U / TLSR_CPU_LEFT_MS) / full_per_ms, 100U);
#endif
}

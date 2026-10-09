/*
 * Stand-in for the host test: the interrupt save and restore tc32_rng uses
 * (no interrupts on the host).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/util.h>
static inline uint8_t tlsr827x_irq_save(void)
{
	return 1U;
}
static inline void tlsr827x_irq_restore(uint8_t key)
{
	(void)key;
}

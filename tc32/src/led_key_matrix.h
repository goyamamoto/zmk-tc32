/*
 * The backlight of a "cidoo,led-key-matrix" (src/led_key_matrix.c), for code
 * that must keep it dark. Without the driver these do nothing.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TC32_LED_KEY_MATRIX_H_
#define TC32_LED_KEY_MATRIX_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#ifdef CONFIG_CIDOO_LED_KEY_MATRIX
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define LKM_NODE      DT_COMPAT_GET_ANY_STATUS_OKAY(cidoo_led_key_matrix)
#define LKM_GPIO_OEN  2U /* 1: output disabled */
#define LKM_GPIO_OUT  3U
#define LKM_GPIO_FUNC 6U /* 1: GPIO */
#define LKM_PWM_PAD(node, prop, idx)                                                               \
	{DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(node, prop, idx)), BIT(DT_GPIO_PIN_BY_IDX(node, prop, idx))},

struct lkm_colour_pads {
	uint8_t oen[DT_PROP_LEN(LKM_NODE, pwm_gpios)];
	uint8_t out[DT_PROP_LEN(LKM_NODE, pwm_gpios)];
};

/*
 * The backlight's colour lines (pwm-gpios) as GPIO outputs driven low, the
 * level they hold while the chip suspends (USB suspend, BLE's low power,
 * deep sleep), after led_key_matrix_blank() has stopped the PWM; the pads' output enable and
 * level are kept in *saved. led_key_matrix_colour_pads_back() puts them back
 * to their PWM function (the PWM still stopped) with what was kept.
 * Interrupts locked for both.
 */
static inline void led_key_matrix_colour_pads_low(struct lkm_colour_pads *saved)
{
	static const struct {
		uint32_t base;
		uint8_t bit;
	} pads[] = {DT_FOREACH_PROP_ELEM(LKM_NODE, pwm_gpios, LKM_PWM_PAD)};

	for (size_t i = 0; i < ARRAY_SIZE(pads); i++) {
		const uint32_t b = pads[i].base;
		const uint8_t bit = pads[i].bit;

		saved->oen[i] = sys_read8(b + LKM_GPIO_OEN) & bit;
		saved->out[i] = sys_read8(b + LKM_GPIO_OUT) & bit;
		/* the level first, then the output, then the GPIO function */
		sys_write8(sys_read8(b + LKM_GPIO_OUT) & (uint8_t)~bit, b + LKM_GPIO_OUT);
		sys_write8(sys_read8(b + LKM_GPIO_OEN) & (uint8_t)~bit, b + LKM_GPIO_OEN);
		sys_write8(sys_read8(b + LKM_GPIO_FUNC) | bit, b + LKM_GPIO_FUNC);
	}
}

static inline void led_key_matrix_colour_pads_back(const struct lkm_colour_pads *saved)
{
	static const struct {
		uint32_t base;
		uint8_t bit;
	} pads[] = {DT_FOREACH_PROP_ELEM(LKM_NODE, pwm_gpios, LKM_PWM_PAD)};

	for (size_t i = 0; i < ARRAY_SIZE(pads); i++) {
		const uint32_t b = pads[i].base;
		const uint8_t bit = pads[i].bit;

		/* the PWM function first (the channel stopped keeps the line low), then the rest */
		sys_write8(sys_read8(b + LKM_GPIO_FUNC) & (uint8_t)~bit, b + LKM_GPIO_FUNC);
		sys_write8((sys_read8(b + LKM_GPIO_OEN) & (uint8_t)~bit) | saved->oen[i], b + LKM_GPIO_OEN);
		sys_write8((sys_read8(b + LKM_GPIO_OUT) & (uint8_t)~bit) | saved->out[i], b + LKM_GPIO_OUT);
	}
}

/*
 * Every PWM output low and every column high, at once. For code about to
 * hold interrupts off for long (a flash write, the chip's suspend): the
 * matrix interrupt stops with it, and the column lit at that moment would
 * stay lit. Call with interrupts locked; the next slot lights the backlight
 * again. Does nothing before the driver has started.
 */
void led_key_matrix_blank(void);

/* While hidden the backlight stays dark (the host sleeps); the keys are still scanned. */
void led_key_matrix_hide(bool hidden);

/*
 * Submits work to queue every half_ms half milliseconds of the matrix's
 * clock (the effect ticks are counted in the matrix's interrupt); 0
 * stops. The clock stops while the chip suspends.
 */
void led_key_matrix_tick(struct k_work_q *queue, struct k_work *work, uint32_t half_ms);

/*
 * Asleep: Timer1 stopped (no key scan, no backlight, no USB poll from it),
 * every column high, and each row a wake pad for the chip's suspend on the
 * level opposite to the one it reads (a key going down, or up). Awake again:
 * the pads off and the timer started with a key scan. The low-power state
 * while BLE is connected and idle (TLSR_BLE_LOW_POWER).
 */
#if IS_ENABLED(CONFIG_TLSR_BLE_LOW_POWER)
void led_key_matrix_sleep(bool sleep);
#else
static inline void led_key_matrix_sleep(bool sleep)
{
	(void)sleep;
}
#endif
#else
struct lkm_colour_pads {
	uint8_t unused;
};

static inline void led_key_matrix_colour_pads_low(struct lkm_colour_pads *saved)
{
	(void)saved;
}

static inline void led_key_matrix_colour_pads_back(const struct lkm_colour_pads *saved)
{
	(void)saved;
}

static inline void led_key_matrix_blank(void)
{
}

#if IS_ENABLED(CONFIG_LED_DRIVER_SPI)
void v75pro_backlight_hide(bool hidden);

static inline void led_key_matrix_hide(bool hidden)
{
	v75pro_backlight_hide(hidden);
}
#else
static inline void led_key_matrix_hide(bool hidden)
{
	(void)hidden;
}
#endif

static inline void led_key_matrix_tick(struct k_work_q *queue, struct k_work *work,
				       uint32_t half_ms)
{
	(void)queue;
	(void)work;
	(void)half_ms;
}

static inline void led_key_matrix_sleep(bool sleep)
{
	(void)sleep;
}
#endif

#endif /* TC32_LED_KEY_MATRIX_H_ */

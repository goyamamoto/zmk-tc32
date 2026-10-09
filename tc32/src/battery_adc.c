/*
 * The battery of a "cidoo,battery" board, measured as follows
 * (dts/bindings/misc/cidoo,battery.yaml), for the BLE stack's level
 * (src/ble/ble_battery.c) and the OTA receiver's battery command
 * (src/tlsr_usb_ota.c, 0xff06):
 *
 * - The ADC: the analog settings below with the input against ground, the
 *   ADC reset (reg_rst1 bit 5), 25 us, then DFIFO2 into a 32-byte buffer.
 *   Each of the eight samples is taken when it arrives, or as 0 when none
 *   has come 25 us after the one before (the B87 SDK's
 *   adc_sample_and_get_result waits without a limit). Bit 13 set (below
 *   ground) reads 0. The middle four of the sorted eight are averaged:
 *   mV = (avg * 590 >> 9) + 71, the board's millivolt scale.
 *   The ADC is powered down again after each measurement (analog 0xfc
 *   bit 5); every measurement powers it up first.
 * - The low four bits of the eight samples, as taken, go to
 *   tc32_rng_add_sample() as one 32-bit word (with TC32_RNG).
 * - tc32_rng's ADC source (TC32_RNG_ADC) converts VBAT between two
 *   measurements: its tc32_rng_adc_lock() and tc32_rng_adc_unlock() are
 *   battery_lock here, and it puts every setting back after its use; each
 *   measurement sets the ADC up in full anyway.
 * - The percentage: (mV - empty) * 100 / (full - empty), clamped to 0-100.
 * - The pins, set at the first measurement: power in an input, charging an
 *   input with its pull-up (the devicetree flags).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "ble/tlsr_analog.h"
#include "battery_adc.h"

#define BATTERY DT_COMPAT_GET_ANY_STATUS_OKAY(cidoo_battery)

#define REG_RST1         0x00800061U
#define RST1_ADC         BIT(5)
#define REG_DFIFO2_ADDR  0x00800b08U /* 16 bits; 0xb0b keeps its 0x04 */
#define REG_DFIFO2_SIZE  0x00800b0aU /* 16-byte units - 1 */
#define REG_DFIFO_MODE   0x00800b10U
#define DFIFO_MODE_2     BIT(2)
#define REG_DFIFO2_WPTR  0x00800b1eU
#define REG_SYSTEM_TICK  0x00800740U /* 16 MHz */

#define ANA_ADC_PGA_CTRL 0xfcU
#define ADC_POWER_DOWN   BIT(5)

#define SAMPLES     8U
#define SAMPLE_WAIT (25U * 16U)

static const struct gpio_dt_spec power_in = GPIO_DT_SPEC_GET(BATTERY, power_gpios);
static const struct gpio_dt_spec charging_in = GPIO_DT_SPEC_GET(BATTERY, charging_gpios);

static volatile uint32_t dma_buf[SAMPLES] __aligned(16);
static uint16_t last_avg;
static bool pins_set;
K_MUTEX_DEFINE(battery_lock);

static uint32_t systick(void)
{
	return sys_read32(REG_SYSTEM_TICK);
}

static void adc_setup(void)
{
	unsigned int key = irq_lock();

	tlsr_analog_write(ANA_ADC_PGA_CTRL, tlsr_analog_read(ANA_ADC_PGA_CTRL) | ADC_POWER_DOWN);
	tlsr_analog_write(0xf4, 0x05);
	tlsr_analog_write(0xf2, 0x24);
	tlsr_analog_write(0xef, 0xf0);
	tlsr_analog_write(0xf1, 0x0a);
	tlsr_analog_write(0xec, 0x43);
	tlsr_analog_write(0xeb, (uint8_t)((DT_PROP(BATTERY, adc_input) & 0xf) << 4) | 0x0fU);
	tlsr_analog_write(0xea, 0x02);
	tlsr_analog_write(0xfa, (tlsr_analog_read(0xfa) & 0xc0U) | 0x3dU);
	tlsr_analog_write(0xee, 0x01);
	tlsr_analog_write(0xfa, (tlsr_analog_read(0xfa) & 0x3fU) | (3U << 6));
	tlsr_analog_write(0xf9, tlsr_analog_read(0xf9) | 0x10U);
	tlsr_analog_write(ANA_ADC_PGA_CTRL, tlsr_analog_read(ANA_ADC_PGA_CTRL) & (uint8_t)~ADC_POWER_DOWN);
	irq_unlock(key);
}

static void adc_power_down(void)
{
	unsigned int key = irq_lock();

	tlsr_analog_write(ANA_ADC_PGA_CTRL, tlsr_analog_read(ANA_ADC_PGA_CTRL) | ADC_POWER_DOWN);
	irq_unlock(key);
}

/* The millivolts on the board's scale, or -1 when the buffer is not where DFIFO2 can write. */
static int measure_mv(void)
{
	uint16_t sorted[SAMPLES];
	uintptr_t buf = (uintptr_t)dma_buf;

	if ((buf >> 16) != 0x84U) {
		return -1;
	}
	adc_setup();
	uint8_t rst = sys_read8(REG_RST1);

	sys_write8(rst | RST1_ADC, REG_RST1);
	sys_write8(rst & (uint8_t)~RST1_ADC, REG_RST1);
	for (unsigned int i = 0; i < SAMPLES; i++) {
		dma_buf[i] = 0;
	}

	uint32_t t0 = systick();

	while (systick() - t0 <= SAMPLE_WAIT) {
	}
	sys_write16((uint16_t)buf, REG_DFIFO2_ADDR);
	sys_write8(sizeof(dma_buf) / 16U - 1U, REG_DFIFO2_SIZE);
	sys_write16(0, REG_DFIFO2_WPTR);
	sys_write8(sys_read8(REG_DFIFO_MODE) | DFIFO_MODE_2, REG_DFIFO_MODE);

	t0 = systick();
	uint32_t lsbs = 0U;

	for (unsigned int i = 0; i < SAMPLES; i++) {
		uint32_t raw;

		while ((raw = dma_buf[i]) == 0U && systick() - t0 <= SAMPLE_WAIT) {
		}
		t0 = systick();
		uint16_t s = (raw & BIT(13)) != 0U ? 0U : (uint16_t)(raw & 0x1fffU);
		int j = (int)i - 1;

		lsbs = (lsbs << 4) | (s & 0x0fU);

		while (j >= 0 && sorted[j] > s) {
			sorted[j + 1] = sorted[j];
			j--;
		}
		sorted[j + 1] = s;
	}
	sys_write8(sys_read8(REG_DFIFO_MODE) & (uint8_t)~DFIFO_MODE_2, REG_DFIFO_MODE);
	adc_power_down();
	if (IS_ENABLED(CONFIG_TC32_RNG)) {
		tc32_rng_add_sample(lsbs, TC32_RNG_SRC_ADC_BATTERY);
	}

	uint32_t avg = ((uint32_t)sorted[2] + sorted[3] + sorted[4] + sorted[5]) >> 2;

	last_avg = (uint16_t)avg;
	return (int)((avg * 590U) >> 9) + 71;
}

static uint8_t percent(int mv)
{
	const int full = DT_PROP(BATTERY, full_millivolt);
	const int empty = DT_PROP(BATTERY, empty_millivolt);

	if (mv <= empty) {
		return 0;
	}
	if (mv >= full) {
		return 100;
	}
	return (uint8_t)((mv - empty) * 100 / (full - empty));
}

/* The charger pins as inputs, once (under battery_lock). */
static void pins_up(void)
{
	if (!pins_set) {
		pins_set = true;
		(void)gpio_pin_configure_dt(&power_in, GPIO_INPUT);
		(void)gpio_pin_configure_dt(&charging_in, GPIO_INPUT);
	}
}

bool cidoo_battery_power_in(void)
{
	bool power;

	k_mutex_lock(&battery_lock, K_FOREVER);
	pins_up();
	power = gpio_pin_get_dt(&power_in) > 0;
	k_mutex_unlock(&battery_lock);
	return power;
}

int cidoo_battery_read(struct cidoo_battery_reading *out)
{
	int mv;

	k_mutex_lock(&battery_lock, K_FOREVER);
	pins_up();
	mv = measure_mv();
	out->raw = last_avg;
	k_mutex_unlock(&battery_lock);
	if (mv < 0) {
		return -EIO;
	}
	out->mv = (uint16_t)mv;
	out->percent = percent(mv);
	out->power = gpio_pin_get_dt(&power_in) > 0;
	out->charging = out->power && gpio_pin_get_dt(&charging_in) > 0;
	return 0;
}

uint16_t cidoo_battery_empty_mv(void)
{
	return DT_PROP(BATTERY, empty_millivolt);
}

#if defined(CONFIG_TC32_RNG_ADC)
int tc32_rng_adc_lock(void)
{
	return k_mutex_lock(&battery_lock, K_FOREVER);
}

void tc32_rng_adc_unlock(void)
{
	(void)k_mutex_unlock(&battery_lock);
}
#endif

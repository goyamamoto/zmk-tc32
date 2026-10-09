/*
 * The battery of a "cidoo,battery" board (src/battery_adc.c): one
 * measurement (the ADC, millivolts and percentage), and its charger pins.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TC32_BATTERY_ADC_H_
#define TC32_BATTERY_ADC_H_

#include <stdbool.h>
#include <stdint.h>

struct cidoo_battery_reading {
	uint16_t mv;      /* millivolts on the board's scale: (raw * 590 >> 9) + 71 */
	uint16_t raw;     /* the middle four of eight ADC samples, averaged */
	uint8_t percent;  /* the percentage of mv between the empty and full points */
	bool power;       /* power in (the V21: PD7 high) */
	bool charging;    /* power in and charging (the V21: PD6 high); false once complete */
};

/*
 * Measures now (under a mutex: the BLE thread and the OTA thread both ask).
 * The first call sets the charger pins up. -EIO when the DMA buffer is out of
 * DFIFO2's reach (never on the TLSR8278's RAM).
 */
int cidoo_battery_read(struct cidoo_battery_reading *out);

/* Whether power comes in (the cable), with no measurement. */
bool cidoo_battery_power_in(void);

/* The empty point, in the board's millivolt scale. */
uint16_t cidoo_battery_empty_mv(void);

#endif /* TC32_BATTERY_ADC_H_ */

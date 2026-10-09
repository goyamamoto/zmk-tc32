/*
 * The status display (CONFIG_TLSR_STATUS_DISPLAY): the keyboard's state shown
 * on its LEDs for a few seconds after the &sts key - the battery, the CPU
 * left over, the link modes and the Bluetooth profiles. This part gathers the
 * state; a board's part (status_board_show()) decides which LEDs show what.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TC32_STATUS_DISPLAY_H_
#define TC32_STATUS_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>

enum status_link {
	STATUS_LINK_WIRED,
	STATUS_LINK_BLE,
	STATUS_LINK_P24,
};

struct status_snapshot {
	bool battery_known;     /* a measurement was made */
	uint8_t battery_percent;
	bool power_in;          /* the cable supplies power */
	bool charging;          /* power in and the charger charging */
	uint8_t cpu_left;       /* percent the lowest-priority thread gets; 0xff until measured */
	enum status_link link;  /* which link runs, from the mode switch read at boot */
	uint8_t ble_active;     /* the selected profile */
	uint8_t ble_bonded;     /* a bit per profile with a bond */
	bool ble_connected;     /* a central is connected */
	bool ble_ready;         /* and encrypted with the keyboard report's notifications on */
	bool ble_pairing;       /* advertising discoverable: the profile has no bond */
	bool p24_linked;        /* the 2.4G link is up */
	bool p24_pairing;       /* the 2.4G link is pairing */
	bool mode_known;        /* the keymap has a Mac mode (a kept layer, layer_keep.c) */
	bool mac_mode;          /* and it is on */
};

/* The &sts key: the state gathered, shown now and again once the CPU figure is in, and hidden
 * after CONFIG_TLSR_STATUS_DISPLAY_MS. */
void status_display_show(void);

/* The board's part: the snapshot on the LEDs (called again when it changes), and off. */
void status_board_show(const struct status_snapshot *s);
void status_board_hide(void);

#endif /* TC32_STATUS_DISPLAY_H_ */

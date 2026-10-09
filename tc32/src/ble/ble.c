/*
 * This firmware's own BLE stack, run from its own thread in the BT
 * position of the mode switch: advertising (ble_adv.c) until a central
 * connects, the connection (ble_conn.c, ble_ll_ctrl.c) until it ends, then
 * advertising again.
 *
 * - Mode switch ("cidoo,mode-switch" in the board's devicetree): the BT and
 *   2.4G position pins are read three times with their pull-ups, then left
 *   with input, output and pull off (board.c leaves them so). All three
 *   reads must agree on BT for this stack to run. The V21: PC4 low is BT,
 *   PB6 low 2.4G. The V75 Pro: PA0 low is BT, PB6 low 2.4G, with PA7 (the
 *   SWS pin) driven low only while the switch is read and then given its
 *   level, output enable and function back.
 * - The position at boot is the one the switch rests in: it is read every
 *   10 ms until it has read the same for 100 ms, or for 300 ms when that is
 *   wired, since a switch on its way from one end to the other reads as wired
 *   in the middle (at most 2 s, then the last reading).
 * - The switch moved while running: the position read at boot decides which
 *   link runs and which transport ZMK prefers, so a new position reboots the
 *   chip (switch_poll()), and the boot after it reads the position again.
 *   The switch is read every 50 ms while power comes in from the cable, and
 *   every second on the battery, where moving it passes the wired position,
 *   which cuts the battery off. The watch starts once a reading agrees with
 *   the one made at boot (the first comes 10 ms after it), or 2 s after the
 *   boot: a switch moved away before the first reading is still followed,
 *   and pads that read differently at boot and later reboot the chip once
 *   every 2 s at most, the power-on chord and USB still working between.
 *   Once it has started, two readings in a row that differ from the boot's
 *   reboot it, whatever positions the switch passed between them. The
 *   reboot of a confirmed image is planned (not counted), as a wake from
 *   deep sleep is.
 * - Per-unit values (flash 0x76000, read only): bytes 0-3 are the ID the
 *   addresses are built from, byte 9 the BLE TX power level
 *   (TLSR_BLE_TX_POWER_DEFAULT, 0x2b, when blank), byte 13 non-zero (also
 *   when blank) sets analog 0x8a bit 7; when it is 0, analog 0x8a bits 5:0
 *   get bits 5:0 of the byte at 0x77000, unless that is blank (analog 0x8a
 *   not documented). If the ID is blank, the address changes at every boot
 *   (tc32_rng_fill()).
 * - Profiles (TLSR_BLE_PROFILES): each has its own address and bond
 *   (ble_bond.c). ZMK's &bt behaviour asks through ble_zmk.c to select a
 *   profile, clear bonds or disconnect; the thread ends a connection first
 *   (LL_TERMINATE_IND 0x13; the connection ends when the central has
 *   acknowledged it, or at the supervision timeout; no report goes out
 *   after it), then applies the request and advertises with the profile's
 *   address.
 * - An advertising event begins TLSR_BLE_ADV_INTERVAL_MIN + rand % (MAX -
 *   MIN + 1) units of 0.625 ms after the one before began (20-25 ms by
 *   default), or later when the thread wakes late; rand is a word of
 *   tc32_rng_fill().
 * - Random numbers come from tc32_rng (zephyr-tc32): the pairing's Srand and
 *   the distributed LTK, EDIV and Rand from tc32_rng_get_key_material()
 *   (ble_smp.c), the SKDs and IVs of an encryption start from
 *   tc32_rng_get() (ble_link.c). Before each advertising run the thread
 *   calls tc32_rng_collect(), which returns at once when the generator is
 *   ready (a seed record kept from the boot before, or an earlier
 *   collection) and otherwise collects first, once: about 40 ms on a boot
 *   without a seed record from the 32 kHz jitter and the ADC's noise
 *   together. While the generator is not ready (a collection gave up: its
 *   time limit, or a 32 kHz count that stands still while the ADC credits
 *   nothing; or its AES known-answer test failed) the thread does not
 *   advertise: it collects again every RNG_RETRY_MS, and the advertising
 *   time limits run on as if no host came (deep sleep, or the pause while
 *   a USB host has the keyboard). A request the generator refuses anyway
 *   (with TLSR_BLE_RNG_COLLECT off) ends the pairing with Pairing Failed or
 *   the encryption start with LL_REJECT_IND; nothing weaker takes its place.
 * - With no host: deep sleep after 20 s of advertising to a bonded host or
 *   60 s discoverable (ble_sleep.c). While a USB host has the keyboard over
 *   the cable (ble_usb_host(): ZMK's HID ready and power in) there is no
 *   deep sleep, so the advertising stops there instead, until a key or the
 *   knob; once the cable is pulled (no power in), the deep sleep comes.
 * - With TLSR_USB_WIRED_POSITION_ONLY, USB is enabled only when the switch
 *   reads wired at boot, after the reading settled: in the BT and 2.4G
 *   positions the cable only brings power and no USB host has the keyboard,
 *   so the deep sleep above comes with the cable in too. With
 *   TLSR_USB_ON_REQUEST, &usb_on enables USB there later
 *   (ble_usb_request()); from then on a host on the cable has the keyboard,
 *   as above, though no key report goes to it.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/random/tc32_rng.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "ble_flash.h"
#include "ble_internal.h"
#include "led_key_matrix_effects.h"
#include "tlsr_analog.h"
#include <tlsr_radio.h>
#if IS_ENABLED(CONFIG_BATTERY_ADC)
#include "battery_adc.h"
#endif
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD)
#include "tlsr_slots.h"
#endif
#if IS_ENABLED(CONFIG_TLSR_USB_WIRED_POSITION_ONLY)
#include <zmk/usb.h>
#endif
#if IS_ENABLED(CONFIG_TLSR_USB_POLL_TIMER)
#include "usb_poll_timer.h"
#endif

#define GPIO_OEN  2U /* 1: output disabled */
#define GPIO_OUT  3U
#define GPIO_FUNC 6U /* 1: GPIO */

#define ANA_8A  0x8aU

#define UNIT_DATA     0x76000U
#define UNIT_TX_LEVEL 9U
#define UNIT_8A_BIT7 13U
#define CALIB_8A     0x77000U

#define ADV_UNIT_TICKS (625U * TLSR_RADIO_TICKS_PER_US)

/* Connection parameter requests per connection: the first, and the fallback after a rejection. */
#define PARAM_REQUESTS      (IS_ENABLED(CONFIG_TLSR_BLE_CONN_FALLBACK) ? 2U : 1U)
/* TGAP(conn_param_timeout): no request sooner than this after a rejection (Core Vol 3 Part C 9.3.9.2). */
#define CONN_PARAM_RETRY_MS 30000U

enum mode_position {
	MODE_WIRED,
	MODE_BLE,
	MODE_24G,
};

#if DT_HAS_CHOSEN(zephyr_flash_controller)
#define BLE_FLASH_NODE DT_CHOSEN(zephyr_flash_controller)
#else
#define BLE_FLASH_NODE DT_CHOSEN(zephyr_flash)
#endif

static const struct device *const flash_dev = DEVICE_DT_GET(BLE_FLASH_NODE);

struct cidoo_ble_stats cidoo_ble_stats;

#if defined(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
/*
 * No mode switch: the cable's power (the battery node's power-in pin, high)
 * and the link chosen by key and kept with the profiles decide. With power
 * in: LINK_USB is the wired mode, LINK_BLE and LINK_P24 the wireless links
 * (the cable then only brings power). Without it only the wireless links
 * can run: LINK_USB falls back to BLE. The pin is read three times 10 us
 * apart as the switch pins are, and the choice is read with it, so a key
 * that changes the choice is followed by the watch as a moved switch is.
 */
static enum mode_position link_mode(bool power)
{
	struct ble_profile_state st;

	ble_profile_state_get(&st);
	if (st.link == LINK_P24 && IS_ENABLED(CONFIG_TLSR_P24)) {
		return MODE_24G;
	}
	return power && st.link == LINK_USB ? MODE_WIRED : MODE_BLE;
}

static enum mode_position read_mode_switch(void)
{
	int power = 0;

	for (int i = 0; i < 3; i++) {
		power += cidoo_battery_power_in() ? 1 : 0;
		k_busy_wait(10);
	}
	return link_mode(power == 3);
}
#elif !defined(CONFIG_TLSR_BLE_IGNORE_MODE_SWITCH)
#define MODE_SWITCH DT_COMPAT_GET_ANY_STATUS_OKAY(cidoo_mode_switch)
BUILD_ASSERT(DT_NODE_EXISTS(MODE_SWITCH),
	     "TLSR_BLE needs the board's \"cidoo,mode-switch\" (or TLSR_BLE_IGNORE_MODE_SWITCH)");

static const struct gpio_dt_spec switch_ble = GPIO_DT_SPEC_GET(MODE_SWITCH, ble_gpios);
static const struct gpio_dt_spec switch_rf24 = GPIO_DT_SPEC_GET(MODE_SWITCH, rf24_gpios);

#if DT_NODE_HAS_PROP(MODE_SWITCH, common_gpios)
#define COMMON_PORT DT_REG_ADDR(DT_GPIO_CTLR(MODE_SWITCH, common_gpios))
#define COMMON_BIT  BIT(DT_GPIO_PIN(MODE_SWITCH, common_gpios))

struct common_saved {
	uint8_t oen, out, func;
};

/* The common pin low: level first, then the GPIO function, then the output enable. */
static void common_low(struct common_saved *s)
{
	unsigned int key = irq_lock();

	s->oen = sys_read8(COMMON_PORT + GPIO_OEN);
	s->out = sys_read8(COMMON_PORT + GPIO_OUT);
	s->func = sys_read8(COMMON_PORT + GPIO_FUNC);
	sys_write8(s->out & (uint8_t)~COMMON_BIT, COMMON_PORT + GPIO_OUT);
	sys_write8(s->func | COMMON_BIT, COMMON_PORT + GPIO_FUNC);
	sys_write8(s->oen & (uint8_t)~COMMON_BIT, COMMON_PORT + GPIO_OEN);
	irq_unlock(key);
}

/* Output off first, then its own function and level again. */
static void common_restore(const struct common_saved *s)
{
	unsigned int key = irq_lock();

	sys_write8(sys_read8(COMMON_PORT + GPIO_OEN) | (s->oen & COMMON_BIT), COMMON_PORT + GPIO_OEN);
	sys_write8((sys_read8(COMMON_PORT + GPIO_FUNC) & (uint8_t)~COMMON_BIT) | (s->func & COMMON_BIT),
		   COMMON_PORT + GPIO_FUNC);
	sys_write8((sys_read8(COMMON_PORT + GPIO_OUT) & (uint8_t)~COMMON_BIT) | (s->out & COMMON_BIT),
		   COMMON_PORT + GPIO_OUT);
	irq_unlock(key);
}
#endif

static enum mode_position read_mode_switch(void)
{
	int ble = 0, rf24 = 0;
#if DT_NODE_HAS_PROP(MODE_SWITCH, common_gpios)
	struct common_saved common;

	common_low(&common);
#endif
	/* Input with the pull-up the devicetree flags give */
	(void)gpio_pin_configure_dt(&switch_ble, GPIO_INPUT);
	(void)gpio_pin_configure_dt(&switch_rf24, GPIO_INPUT);
	k_busy_wait(50);
	for (int i = 0; i < 3; i++) {
		bool b = gpio_pin_get_dt(&switch_ble) == 1;
		bool r = gpio_pin_get_dt(&switch_rf24) == 1;

		ble += b && !r;
		rf24 += r && !b;
		k_busy_wait(10);
	}
	/* Input, output and pull off, as board.c leaves them */
	(void)gpio_pin_configure(switch_ble.port, switch_ble.pin, GPIO_DISCONNECTED);
	(void)gpio_pin_configure(switch_rf24.port, switch_rf24.pin, GPIO_DISCONNECTED);
#if DT_NODE_HAS_PROP(MODE_SWITCH, common_gpios)
	common_restore(&common);
#endif
	return ble == 3 ? MODE_BLE : (rf24 == 3 ? MODE_24G : MODE_WIRED);
}
#endif /* the mode switch, or the power-in pin */

#if !defined(CONFIG_TLSR_BLE_IGNORE_MODE_SWITCH)
#define SWITCH_POLL_MS         CONFIG_TLSR_BLE_SWITCH_POLL_MS
#define SWITCH_POLL_BATTERY_MS 1000
#define SWITCH_ARM_MS          10
#define SWITCH_ARM_LATE_MS     2000
#define SWITCH_SETTLE_STEP_MS  10
#define SWITCH_SETTLE_MS       100
#define SWITCH_SETTLE_WIRED_MS 300
#define SWITCH_SETTLE_MAX_MS   2000

/* The position the switch rests in: the same reading for SWITCH_SETTLE_MS, or SWITCH_SETTLE_WIRED_MS for wired. */
static enum mode_position read_mode_switch_settled(void)
{
	int64_t start = k_uptime_get();
	int64_t since = start;
	enum mode_position m = read_mode_switch();

	for (;;) {
		int64_t now = k_uptime_get();

		if (now - since >= (m == MODE_WIRED ? SWITCH_SETTLE_WIRED_MS : SWITCH_SETTLE_MS) ||
		    now - start >= SWITCH_SETTLE_MAX_MS) {
			return m;
		}
		k_sleep(K_MSEC(SWITCH_SETTLE_STEP_MS));

		enum mode_position r = read_mode_switch();

		if (r != m) {
			m = r;
			since = k_uptime_get();
		}
	}
}

static enum mode_position boot_mode;
static bool switch_armed;    /* a reading has agreed with boot_mode */
static uint8_t switch_moved; /* readings in a row that are not boot_mode, once armed */
static struct k_work_delayable switch_work;

static void switch_poll(struct k_work *work)
{
	ARG_UNUSED(work);
	bool power = true;

	if (k_uptime_get() >= SWITCH_ARM_LATE_MS) {
		switch_armed = true; /* moved before any reading agreed: followed, one reboot per 2 s at most */
	}
	if (read_mode_switch() == boot_mode) {
		switch_armed = true;
		switch_moved = 0;
	} else if (switch_armed && ++switch_moved >= 2U) {
		ble_bond_save_db_seen(); /* a database hash a link in progress has set */
		cidoo_backlight_save_pending();
#if IS_ENABLED(CONFIG_TLSR_BOOT_GUARD)
		tlsr_boot_guard_plan_wake(); /* a confirmed image: planned; otherwise counted */
#endif
		sys_reboot(SYS_REBOOT_COLD);
	}
#if IS_ENABLED(CONFIG_BATTERY_ADC)
	/* In the wired position the power comes from the cable: the charger pins stay as board.c leaves them */
	if (boot_mode != MODE_WIRED) {
		power = cidoo_battery_power_in();
	}
#endif
	(void)k_work_schedule(&switch_work,
			      K_MSEC(power || switch_moved != 0U ? SWITCH_POLL_MS : SWITCH_POLL_BATTERY_MS));
}

static void switch_watch(enum mode_position mode)
{
	boot_mode = mode;
	k_work_init_delayable(&switch_work, switch_poll);
	(void)k_work_schedule(&switch_work, K_MSEC(SWITCH_ARM_MS));
}

#if IS_ENABLED(CONFIG_TLSR_USB_WIRED_POSITION_ONLY)
static bool usb_up;

/* USB from now until the power goes, with its 1 ms poll. */
static void usb_start(void)
{
	if (usb_up) {
		return;
	}
	usb_up = true;
#if IS_ENABLED(CONFIG_TLSR_USB_POLL_TIMER)
	usb_poll_timer_run(true);
#endif
	(void)zmk_usb_enable();
}
#endif

#if IS_ENABLED(CONFIG_TLSR_USB_ON_REQUEST)
bool ble_usb_request(void)
{
	if (usb_up) {
		return false;
	}
	usb_start();
	return true;
}

/* HID reports go over USB only when the switch read wired at boot: in the BT and 2.4G positions a USB that was
 * asked for serves the host tools and types nothing. */
bool zmk_usb_reports_allowed(void)
{
	return boot_mode == MODE_WIRED;
}
#endif

#if defined(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
void ble_link_request(uint8_t link)
{
	struct ble_profile_state st;

	if (link == LINK_USB && !cidoo_battery_power_in()) {
		return; /* USB without the cable: nothing to switch to; the choice is left as it is */
	}
	ble_profile_state_get(&st);
	if (st.link != link) {
		st.link = link;
		ble_profile_state_set(&st); /* kept with the profiles, written from ZMK's thread */
	}
	/* The watch (switch_poll) reads the choice with the pin and reboots once the mode changed */
}
#endif
#endif /* !CONFIG_TLSR_BLE_IGNORE_MODE_SWITCH */

/*
 * The links' counters for a host tool: two 16-bit sizes, then cidoo_ble_stats and the 2.4G link's (p24.c), as
 * they are in RAM. n octets from at; what lies past the end is not written.
 */
size_t ble_link_stats_read(size_t at, uint8_t *out, size_t n)
{
	uint8_t head[4];
	size_t p24_len = 0;
	const uint8_t *p24 = NULL;
	size_t done = 0;

#if IS_ENABLED(CONFIG_TLSR_P24)
	p24 = p24_stats(&p24_len);
#endif
	sys_put_le16((uint16_t)sizeof(cidoo_ble_stats), &head[0]);
	sys_put_le16((uint16_t)p24_len, &head[2]);

	const struct {
		const uint8_t *from;
		size_t len;
	} parts[] = {{head, sizeof(head)},
		     {(const uint8_t *)&cidoo_ble_stats, sizeof(cidoo_ble_stats)},
		     {p24, p24_len}};

	for (size_t i = 0; i < ARRAY_SIZE(parts) && done < n; i++) {
		if (at >= parts[i].len) {
			at -= parts[i].len;
			continue;
		}
		size_t k = MIN(n - done, parts[i].len - at);

		memcpy(out + done, parts[i].from + at, k);
		done += k;
		at = 0;
	}
	return done;
}

/* The TX power level from the per-unit data (ble_sleep.c sets it again after a suspend). */
uint8_t ble_tx_level;

static void unit_config(uint8_t id[4], uint8_t *tx_level, uint8_t unit[16])
{
	uint8_t cal = 0xff;

	if (flash_read(flash_dev, UNIT_DATA, unit, 16) != 0) {
		memset(unit, 0xff, 16);
	}
	memcpy(id, unit, 4);
	*tx_level = unit[UNIT_TX_LEVEL] == 0xffU ? CONFIG_TLSR_BLE_TX_POWER_DEFAULT : unit[UNIT_TX_LEVEL];
	if (unit[UNIT_8A_BIT7] == 0U && flash_read(flash_dev, CALIB_8A, &cal, 1) != 0) {
		cal = 0xff;
	}

	unsigned int key = irq_lock();

	if (unit[UNIT_8A_BIT7] != 0U) {
		tlsr_analog_write(ANA_8A, tlsr_analog_read(ANA_8A) | BIT(7));
	} else if (cal != 0xffU) {
		tlsr_analog_write(ANA_8A, (tlsr_analog_read(ANA_8A) & 0xc0U) | (cal & 0x3fU));
	}
	irq_unlock(key);
}

/* Advertising events and flash writes take turns on this (ble_flash.h). */
static K_MUTEX_DEFINE(radio_mutex);
static volatile bool running;

/* Profiles: the state, and the requests ZMK's thread leaves for this one. */
#define PROFILES CONFIG_TLSR_BLE_PROFILES
static uint8_t unit_id[4];
static struct ble_profile_state pstate;
static struct {
	int8_t select;      /* -1: none */
	uint8_t clear;      /* profiles whose bonds go */
	bool disconnect;
} req = {.select = -1};

bool ble_running(void)
{
	return running;
}

uint8_t ble_profile_active(void)
{
	return pstate.active;
}

uint8_t ble_profile_selected(void)
{
	int8_t select = req.select;

	return select >= 0 ? (uint8_t)select : pstate.active;
}

void ble_profile_request(int8_t select, uint8_t clear, bool disconnect)
{
	unsigned int key = irq_lock();

	if (select >= 0) {
		req.select = select;
	}
	req.clear |= clear;
	req.disconnect |= disconnect;
	irq_unlock(key);
	ble_conn_kick();
}

static bool requests_pending(void)
{
	return req.select >= 0 || req.clear != 0U || req.disconnect;
}

static int64_t adv_since;   /* when the advertising under way began */
static bool adv_pairing;    /* discoverable: no bond on the profile */

static bool rng_ready; /* tc32_rng_collect() gave 0: a pairing's keys and a reconnection's session values come */

static void adv_setup(void)
{
	adv_pairing = !ble_bond_valid(pstate.active);
	rng_ready = !IS_ENABLED(CONFIG_TLSR_BLE_RNG_COLLECT) || tc32_rng_collect() == 0;
	ble_adv_init(unit_id, pstate.active, pstate.gen[pstate.active], adv_pairing);
	adv_since = k_uptime_get();
}

void ble_status(struct ble_status *st)
{
	bool p24 = IS_ENABLED(CONFIG_TLSR_P24) && p24_running();

	memset(st, 0, sizeof(*st));
	st->p24 = p24;
	st->ble = running && !p24;
	st->active = pstate.active;
	for (uint8_t p = 0; p < PROFILES; p++) {
		if (ble_bond_valid(p)) {
			st->bonded |= BIT(p);
		}
	}
	st->connected = st->ble && ble_conn_active();
	st->ready = st->connected && ble_att_ready();
	st->pairing = st->ble && !st->connected && adv_pairing;
#if IS_ENABLED(CONFIG_TLSR_P24)
	if (p24) {
		p24_status(&st->p24_linked, &st->p24_pairing);
	}
#endif
}

static bool adv_paused;          /* the thread's: advertising stopped while a USB host has the keyboard */
static volatile bool adv_resume; /* a key or the knob since (ble_adv_activity()) */

#define ADV_PAUSED_POLL_MS 50
#define RNG_RETRY_MS       1000

void ble_adv_activity(void)
{
	adv_resume = true;
}

/*
 * The advertising timeouts: deep sleep after 20 s to a bonded host or 60 s
 * discoverable by default (ble_sleep.c). While a USB host has the keyboard
 * over the cable the advertising stops at that point instead, until a key or
 * the knob (which otherwise does not extend the time); with the cable pulled,
 * the deep sleep follows. Whether to advertise now.
 */
static bool adv_on(void)
{
#if IS_ENABLED(CONFIG_TLSR_BLE_DEEP_SLEEP)
	int64_t now = k_uptime_get();
	int64_t limit = 1000LL * (adv_pairing ? CONFIG_TLSR_BLE_PAIRING_TIMEOUT_S
					      : CONFIG_TLSR_BLE_RECONNECT_TIMEOUT_S);

	if (adv_resume) {
		adv_resume = false;
		if (adv_paused) {
			adv_paused = false;
			adv_since = now;
		}
	}
	if (now - adv_since < limit) {
		return true;
	}
	/*
	 * A USB host on the cable: stop. Once stopped, stay so while power comes
	 * in from the cable (the host enumerating again, a charger), and sleep
	 * when the cable is pulled.
	 */
	if (adv_paused ? ble_cable_power() : ble_usb_host()) {
		adv_paused = true;
		return false;
	}
	ble_deep_sleep();
#endif
	return true;
}

/* Between connections: the requests taken, the profile state saved. */
static void apply_requests(void)
{
	struct ble_profile_state st = pstate;
	unsigned int key = irq_lock();
	int8_t select = req.select;
	uint8_t clear = req.clear;

	req.select = -1;
	req.clear = 0;
	req.disconnect = false;
	irq_unlock(key);

	if (select >= 0 && select < PROFILES) {
		st.active = (uint8_t)select;
	}
	for (uint8_t p = 0; p < PROFILES; p++) {
		if (clear & BIT(p)) {
			ble_bond_clear(p);
			st.gen[p]++; /* a new address: the host's old pairing does not match */
		}
	}
	ble_profile_state_set(&st);
	ble_profile_state_get(&pstate); /* the save may have taken the log's state back in */
}
static uint8_t hold_depth; /* the mutex's owner's */

void ble_flash_radio_hold(bool hold)
{
	if (k_is_pre_kernel() || k_is_in_isr() || !running) {
		return;
	}
	if (hold) {
		(void)k_mutex_lock(&radio_mutex, K_FOREVER);
		if (hold_depth++ == 0U) {
			ble_conn_hold();
		}
	} else if (radio_mutex.owner == k_current_get()) {
		/* only a release whose hold took the mutex: `running` may have
		 * turned on between a write's hold and its release */
		if (--hold_depth == 0U) {
			ble_conn_release();
		}
		(void)k_mutex_unlock(&radio_mutex);
	}
}

void ble_radio_take(void)
{
	(void)k_mutex_lock(&radio_mutex, K_FOREVER);
}

void ble_radio_give(void)
{
	(void)k_mutex_unlock(&radio_mutex);
}

/* The connection's LL data, until it ends. */
static void run_connection(const struct ble_conn_req *req)
{
	bool ending = false;
	bool keys_wait = false; /* a pairing waits for the central's acknowledgement of this side's keys */
	uint32_t end_at = 0;
	uint8_t end_mark = 0;
	/* the connection parameter requests sent: the first, then the fallback after a rejection */
	uint8_t params_sent = CONFIG_TLSR_BLE_CONN_PARAM_DELAY_MS < 0 ? PARAM_REQUESTS : 0U;
	bool params_due = params_sent == 0U;
	uint32_t params_at = k_uptime_get_32() + (uint32_t)MAX(CONFIG_TLSR_BLE_CONN_PARAM_DELAY_MS, 0);

	/* ble_adv_event() has started the connection's events: what they have
	 * received waits in the ring for the loop below. */
	ble_ll_ctrl_reset();
	ble_link_start(req);
	while (ble_conn_active()) {
		const uint8_t *pdu;
		uint8_t llid, len;
		uint16_t event;

		if ((requests_pending() || ble_link_end_reason() != 0U) && !ending) {
			/*
			 * A profile change or disconnect, or a link that has to end (no key after
			 * an encryption pause): LL_TERMINATE_IND to the central, 0x13 or the
			 * link's reason. The connection ends once the central has acknowledged it
			 * (a response the control procedures queue behind it does not hold the
			 * link), or when the supervision timeout has passed since it was queued
			 * (Core Vol 6 Part B 5.1.6); no report goes out after it. A full TX ring
			 * queues it at a later pass.
			 */
			uint8_t ind[2] = {0x02 /* LL_TERMINATE_IND */, ble_link_end_reason()};

			if (ind[1] == 0U) {
				ind[1] = BLE_ERR_REMOTE_TERMINATED;
			}
			if (ble_link_tx(LLID_CONTROL, ind, sizeof(ind)) == 0) {
				ending = true;
				end_mark = ble_conn_tx_mark();
				end_at = k_uptime_get_32() + ble_conn_timeout_ms();
			}
		} else if (ending && (ble_conn_tx_acked(end_mark) ||
				      (int32_t)(k_uptime_get_32() - end_at) >= 0)) {
			ble_conn_terminate(BLE_ERR_LOCAL_TERMINATED);
		} else if (!ending && params_due && (int32_t)(k_uptime_get_32() - params_at) >= 0) {
			/* the connection parameter request, 1 s after the connection by default;
			 * only the TX ring's slots for answers free, or an encryption start or pause
			 * under way, leave it to a later pass */
			if (ble_link_request_conn_params(params_sent > 0U) != -ENOBUFS) {
				params_sent++;
				params_due = false;
			}
		} else if (!ending && !params_due && params_sent < PARAM_REQUESTS &&
			   ble_link_conn_params_result() == 1) {
			/*
			 * The central rejected the first request: the fallback set, no
			 * sooner than TGAP(conn_param_timeout) after the rejection (Core
			 * Vol 3 Part C 9.3.9.2, Appendix A).
			 */
			params_due = true;
			params_at = k_uptime_get_32() + CONN_PARAM_RETRY_MS;
		}
		/* an acknowledgement does not wake the thread: it looks for one every 10 ms */
		(void)ble_conn_wait(ending || keys_wait ? 10 : (ble_low_power_on() ? 1000 : 100));
		while ((pdu = ble_conn_rx_peek(&llid, &len, &event)) != NULL) {
			/* the RX slot is this thread's until ble_conn_rx_done(); the
			 * header's first octet sits two before the payload */
			uint8_t *data = (uint8_t *)pdu;

			if (ble_link_rx(data, data[-2], &len)) {
				if (llid == LLID_CONTROL) {
					ble_ll_ctrl_rx(data, len, event);
				} else if (len > 0U) {
					ble_link_rx_data(data, llid, len);
				}
			}
			ble_conn_rx_done();
		}
		keys_wait = ble_smp_poll();
		if (!ending) {
			ble_zmk_flush();
			ble_battery_poll();
		}
		ble_att_cccd_poll();
		ble_low_power_poll(!ending && ble_att_ready());
	}
	(void)ble_smp_poll(); /* an acknowledgement in the connection's last event */
	ble_low_power_poll(false);
	ble_link_end();
	ble_zmk_flush();
}

static void ble_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	uint8_t id[4];
	uint8_t unit[16];
#if defined(CONFIG_TLSR_BLE_IGNORE_MODE_SWITCH)
	enum mode_position mode = MODE_BLE;
#else
#if defined(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
	ble_bond_load(); /* the wireless link kept with the profiles decides the mode */
#endif
	enum mode_position mode = read_mode_switch_settled();

	switch_watch(mode);
#if IS_ENABLED(CONFIG_TLSR_USB_WIRED_POSITION_ONLY)
	if (mode == MODE_WIRED) {
		usb_start();
	}
#endif
#endif
#if IS_ENABLED(CONFIG_TLSR_P24)
	if (mode == MODE_24G) {
		/* the proprietary 2.4G link in this thread instead (p24.c) */
		ble_zmk_mode_ble();
		running = true;
		unit_config(id, &ble_tx_level, unit);
		p24_run(unit);
	}
#endif
	if (mode != MODE_BLE) {
		return;
	}
	ble_zmk_mode_ble();
	ble_bond_load();
	ble_profile_state_get(&pstate);
	if (pstate.active >= PROFILES) {
		pstate.active = 0;
	}
	running = true;
	unit_config(id, &ble_tx_level, unit);
	if (sys_get_le32(id) == 0xffffffffU) {
		tc32_rng_fill(id, sizeof(id));
	}
	memcpy(unit_id, id, sizeof(unit_id));
	adv_setup();

	tlsr_radio_init_ble_1m();
	tlsr_radio_set_power_level(ble_tx_level);
	ble_conn_init();

	uint32_t span = CONFIG_TLSR_BLE_ADV_INTERVAL_MAX - CONFIG_TLSR_BLE_ADV_INTERVAL_MIN + 1U;

	for (;;) {
		struct ble_conn_req conn_req;
		uint32_t r;
		bool connect;

		tc32_rng_fill(&r, sizeof(r));

		/* The next event is due an interval after this one really began. */
		uint32_t next = tlsr_radio_now() + (CONFIG_TLSR_BLE_ADV_INTERVAL_MIN + r % span) * ADV_UNIT_TICKS;

		if (requests_pending()) {
			apply_requests();
			adv_setup();
		}
		ble_battery_poll();
		if (!adv_on()) {
			k_sleep(K_MSEC(ADV_PAUSED_POLL_MS));
			continue;
		}
		if (!rng_ready) {
			/* No advertising until the generator is ready: a new collection later */
			k_sleep(K_MSEC(RNG_RETRY_MS));
			rng_ready = tc32_rng_collect() == 0;
			continue;
		}
		(void)k_mutex_lock(&radio_mutex, K_FOREVER);
		connect = ble_adv_event(&conn_req);
		(void)k_mutex_unlock(&radio_mutex);
		if (connect) {
			run_connection(&conn_req);
			adv_setup(); /* a pairing may have bonded the profile */
			continue;
		}

		int32_t wait = (int32_t)(next - tlsr_radio_now());

		if (wait > 0) {
			k_sleep(K_USEC((uint32_t)wait / TLSR_RADIO_TICKS_PER_US));
		}
	}
}

K_THREAD_DEFINE(cidoo_ble, CONFIG_TLSR_BLE_STACK_SIZE, ble_thread, NULL, NULL, NULL,
		CONFIG_TLSR_BLE_THREAD_PRIORITY, 0, 0);

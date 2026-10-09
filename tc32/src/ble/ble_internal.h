/*
 * The own BLE stack's parts: ble.c runs the stack in its thread, ble_adv.c
 * advertises and takes a CONNECT_IND, ble_conn.c runs the connection events
 * (interrupt context), ble_ll_ctrl.c answers LL control PDUs, ble_link.c
 * encrypts and dispatches L2CAP, ble_smp.c pairs, ble_bond.c keeps the
 * bonds, ble_att.c is the ATT server and GATT database (all in the thread).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TLSR_BLE_INTERNAL_H_
#define TLSR_BLE_INTERNAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tlsr_ble.h"

#define BLE_ADDR_LEN   6U
#define BLE_PDU_MAX    27U /* no data length extension */
/* The connection's TX ring (ble_conn.c), and the slots in it that only answers take (ble_link.c). */
#define BLE_TX_SLOTS       16U
#define BLE_TX_ANSWER_ROOM 8U
#define BLE_CHANNELS   37U

/* Data channel PDU header, first byte */
#define LLID_CONTINUE 0x01U
#define LLID_START    0x02U
#define LLID_CONTROL  0x03U

/* Error codes (Core Vol 1 Part F) */
#define BLE_ERR_CONN_TIMEOUT       0x08U
#define BLE_ERR_REMOTE_TERMINATED  0x13U
#define BLE_ERR_LOCAL_TERMINATED   0x16U
#define BLE_ERR_UNSUPPORTED_REMOTE 0x1aU
#define BLE_ERR_INVALID_LL_PARAMS  0x1eU
#define BLE_ERR_INSTANT_PASSED     0x28U
#define BLE_ERR_CONN_NOT_ESTABLISHED 0x3eU

/* A CONNECT_IND for this device (Core Vol 6 Part B 2.3.3.1). */
struct ble_conn_req {
	uint8_t peer[BLE_ADDR_LEN];
	uint8_t peer_random;
	uint32_t aa;
	uint32_t crc_init;
	uint8_t win_size;    /* 1.25 ms units */
	uint16_t win_offset; /* 1.25 ms units */
	uint16_t interval;   /* 1.25 ms units */
	uint16_t latency;
	uint16_t timeout;    /* 10 ms units */
	uint8_t chm[5];
	uint8_t hop;
	uint8_t sca;
	uint32_t t_end; /* system tick at the end of the CONNECT_IND */
};

/* What the stack has done, for the simulator's checks (tc32emu
 * zmk_ble_checks.py reads it by symbol). */
struct cidoo_ble_stats {
	uint32_t adv_events;
	uint32_t adv_rx;
	uint32_t adv_tx_timeouts;
	uint32_t connections;
	uint32_t conn_events;
	uint32_t conn_missed;
	uint32_t conn_rx;
	uint32_t conn_rx_dup;
	uint32_t conn_rx_bad;
	uint32_t conn_late;
	uint32_t conn_ended;
	uint32_t last_reason;
	uint32_t ll_ctrl_rx;
	uint32_t ll_ctrl_tx;
	uint32_t data_rx;
	uint32_t l2cap_dropped;
	uint32_t encryptions;
	uint32_t mic_failures;
	uint32_t pairings;
	uint32_t smp_failures;
	uint32_t conn_held;     /* connection events skipped for a flash write */
	uint32_t hold_timeouts; /* a flash write that found an event still on air after 50 ms */
	uint32_t conn_skipped;  /* connection events skipped for peripheral latency */
	uint32_t timer_lat_max; /* the longest an event's timer came after its due time, in system ticks */
	uint32_t conn_extended; /* the timer's waits for a packet being received at an event's end */
	uint32_t adv_refused;   /* CONNECT_INDs from an initiator that is not the profile's bonded host */
	uint32_t conn_rx_timer; /* packets the event timer's handler took, their RF interrupt still pending */
	uint32_t conn_anchor_late; /* first packets handled whose time stamp was too late to be an anchor */
	uint32_t scan_rsps;     /* SCAN_RSPs sent to SCAN_REQs for this address */
	uint32_t ll_ctrl_last;     /* the opcode of the last LL control PDU received */
	uint32_t ll_ctrl_seen;     /* bit n: an LL control PDU with opcode n (0x00-0x1f) was received */
	uint32_t ll_ctrl_answered; /* bit n: one with opcode n was answered by ble_ll_ctrl.c with a control PDU */
};
extern struct cidoo_ble_stats cidoo_ble_stats;

/* ble_adv.c */
/* A profile's device name, in the advertising data and GAP's Device Name:
 * CONFIG_ZMK_KEYBOARD_NAME, a space and the profile's number from 1 (the
 * number of its channel key), so a host tells the profiles apart. Returns
 * its length, with no terminating NUL counted. */
#define BLE_NAME_MAX (sizeof(CONFIG_ZMK_KEYBOARD_NAME) + 2U)
uint8_t ble_device_name(uint8_t profile, char out[BLE_NAME_MAX]);
void ble_adv_init(const uint8_t id[4], uint8_t profile, uint8_t gen, bool discoverable);
void ble_adv_address(uint8_t addr[BLE_ADDR_LEN]);
bool ble_adv_event(struct ble_conn_req *req);

/* ble_conn.c: the thread starts a connection, then waits for its end. */
void ble_conn_init(void);
uint8_t *ble_conn_rx_buf(void);
void ble_conn_start(const struct ble_conn_req *req);
bool ble_conn_active(void);
uint8_t ble_conn_reason(void);
/* LL data: one received PDU at a time, in order (thread). */
const uint8_t *ble_conn_rx_peek(uint8_t *llid, uint8_t *len, uint16_t *event);
void ble_conn_rx_done(void);
int ble_conn_tx(uint8_t llid, const uint8_t *data, uint8_t len);
/* PDUs the TX ring takes now. */
uint8_t ble_conn_tx_room(void);
uint16_t ble_conn_event_counter(void);
/* LL control procedures with an instant, applied by ble_conn.c. */
/* rx_event: the event the PDU came in; invalid parameters end the link (0x1e). */
void ble_conn_update(uint8_t win_size, uint16_t win_offset, uint16_t interval, uint16_t latency,
		     uint16_t timeout, uint16_t instant, uint16_t rx_event);
void ble_conn_channel_map(const uint8_t chm[5], uint16_t instant, uint16_t rx_event);
void ble_conn_terminate(uint8_t reason);
/* ble_conn.c gives this when an event received data or the connection ended. */
int ble_conn_wait(int32_t timeout_ms);
/* A mark after the PDUs queued so far, and whether the central has acknowledged them all. */
uint8_t ble_conn_tx_mark(void);
bool ble_conn_tx_acked(uint8_t mark);
/* The connection's supervision timeout, in ms. */
uint32_t ble_conn_timeout_ms(void);
/* Wakes ble_conn_wait() (ZMK queued a report). */
void ble_conn_kick(void);
/* Peripheral latency on (the low-power state) or off (every event, the next one at once). */
void ble_conn_latency(bool on);
/* No event on air (the radio idle until the next one's timer). */
bool ble_conn_between_events(void);
/* No connection event starts from hold until release; hold returns once none is on air. */
void ble_conn_hold(void);
void ble_conn_release(void);

/* ble_ll_ctrl.c */
void ble_ll_ctrl_reset(void);
void ble_ll_ctrl_rx(const uint8_t *pdu, uint8_t len, uint16_t event);

/* ble_link.c */
void ble_link_start(const struct ble_conn_req *req);
void ble_link_end(void);
void ble_link_addresses(uint8_t peer[BLE_ADDR_LEN], uint8_t *peer_random, uint8_t own[BLE_ADDR_LEN]);
bool ble_link_encrypted(void);
/* Encrypted with a bond's key, or bonded during this connection. */
bool ble_link_bonded(void);
void ble_link_set_bonded(void);
/* The key of the pairing in progress, for the LL_ENC_REQ with EDIV 0 and Rand 0 that follows. */
void ble_link_set_pairing_key(const uint8_t ltk[16], bool authenticated);
/* The profile's bond is the connected central's (ble_bond_peer_known()). */
bool ble_link_peer_bonded(void);
/* Encrypted with a key from a pairing that protected against MITM (Passkey Entry). */
bool ble_link_authenticated(void);
/* The central has sent LL_ENC_REQ on this link. */
bool ble_link_enc_requested(void);
/* Non-zero: the link is to end with LL_TERMINATE_IND and this reason. */
uint8_t ble_link_end_reason(void);
int ble_link_tx(uint8_t llid, const uint8_t *data, uint8_t len);
bool ble_link_rx(uint8_t *pdu, uint8_t hdr0, uint8_t *len);
void ble_link_rx_data(const uint8_t *pdu, uint8_t llid, uint8_t len);
void ble_link_enc_req(const uint8_t *pdu, uint8_t len);
/* The connection parameters TLSR_BLE_CONN_* ask for (fallback: TLSR_BLE_CONN_FALLBACK_*), as an L2CAP
 * request (0, or -ENOBUFS to ask again later) */
int ble_link_request_conn_params(bool fallback);
/* The central's answer to the last request: -1 none yet, 0 accepted, 1 rejected. */
int ble_link_conn_params_result(void);
void ble_link_start_enc_rsp(void);
bool ble_link_pause_enc_req(void);
void ble_link_pause_enc_rsp(void);
/* An L2CAP frame: an answer, or a report (an ATT notification or indication: -ENOBUFS while only the TX ring's slots
 * for answers are free). */
int ble_l2cap_send(uint16_t cid, const uint8_t *data, uint8_t len);

/* ble_smp.c */
void ble_smp_reset(void);
void ble_smp_rx(const uint8_t *pdu, uint8_t len);
void ble_smp_encrypted(bool with_pairing_key);
/* The BLE thread, every pass and once more when the connection has ended: the pairing ends once the
 * central has acknowledged this side's keys. True while that acknowledgement is waited for. */
bool ble_smp_poll(void);
/* Passkey Entry (ble_passkey.c): a passkey is being asked for; one key of it: a digit (0-9), or Enter,
 * Backspace, Escape. */
#define BLE_PASSKEY_ENTER     10U
#define BLE_PASSKEY_BACKSPACE 11U
#define BLE_PASSKEY_ESCAPE    12U
bool ble_smp_passkey_wanted(void);
void ble_smp_passkey_key(uint8_t key);

/* ble_bond.c: a central's keys, on-air octet order */
struct ble_bond {
	bool valid;
	uint8_t peer[BLE_ADDR_LEN]; /* the address it paired from */
	uint8_t peer_random;
	uint8_t id_addr[BLE_ADDR_LEN]; /* its identity address, if it gave one */
	uint8_t id_random;
	uint8_t irk[16];
	uint8_t ltk[16];            /* the one this side distributed */
	uint16_t ediv;
	uint8_t rand[8];
	uint8_t cccd;               /* the notifications the client turned on (BLE_CCCD_*) */
	uint8_t flags;              /* BLE_BOND_* */
};
#define BLE_BOND_SC            0x01U /* from a Secure Connections pairing (every usable bond) */
#define BLE_BOND_AUTHENTICATED 0x02U /* that pairing protected against MITM (Passkey Entry) */
/* A bonded client's CCCDs, kept with its bond (Core Vol 3 Part G 3.3.3.3). */
#define BLE_CCCD_BATTERY  0x01U
#define BLE_CCCD_KEYBOARD 0x02U
#define BLE_CCCD_CONSUMER 0x04U
#define BLE_CCCD_SERVICE_CHANGED 0x08U /* indications of Service Changed */
#define BLE_CCCD_BOOT_KEYBOARD 0x10U /* the boot keyboard input report */
/* The active profile and each profile's address generation. */
struct ble_profile_state {
	uint8_t active;
	uint8_t gen[CONFIG_TLSR_BLE_PROFILES];
#if IS_ENABLED(CONFIG_TLSR_BLE_MODE_FROM_POWER_IN)
	uint8_t link; /* the link chosen by key on a board without a mode switch: LINK_USB, LINK_BLE or LINK_P24 */
#endif
};
#define LINK_USB 0U
#define LINK_BLE 1U
#define LINK_P24 2U
/* ble.c (TLSR_BLE_MODE_FROM_POWER_IN): keep this link; running another, reboot into it (USB needs the cable). */
void ble_link_request(uint8_t link);
void ble_bond_load(void);
/* A new bond, and the hash of the database its host discovered (ble_att_db_hash()). */
void ble_bond_store(uint8_t profile, const struct ble_bond *bond, uint32_t db_hash);
void ble_bond_clear(uint8_t profile);
/* The profile's bond keeps these CCCD bits; true when a record was written (they changed). */
bool ble_bond_set_cccd(uint8_t profile, uint8_t bits);
uint8_t ble_bond_cccd(uint8_t profile);
bool ble_bond_valid(uint8_t profile);
/* The hash of the database the profile's host last saw (0: none kept), kept with the profile state. */
uint32_t ble_bond_db_seen(uint8_t profile);
void ble_bond_set_db_seen(uint8_t profile, uint32_t hash);
/* A hash ble_bond_store() or ble_bond_set_db_seen() changed goes to the log (at the link's end, or before the
 * mode switch's reboot). */
void ble_bond_save_db_seen(void);
bool ble_bond_find_ltk(uint8_t profile, uint16_t ediv, const uint8_t rand[8], uint8_t ltk[16],
		       bool *authenticated);
/* The address is the bonded host's: the one it paired from, its identity address, or a
 * resolvable private address its IRK resolves. */
bool ble_bond_peer_known(uint8_t profile, const uint8_t addr[BLE_ADDR_LEN], bool random);
void ble_profile_state_get(struct ble_profile_state *st);
/* The caller's state saved: the active profile as given, each generation as the difference from the
 * state ble_profile_state_get() last gave the caller. */
void ble_profile_state_set(const struct ble_profile_state *st);

/* ble.c: the state the status display shows (status_display.c), read from any thread. */
struct ble_status {
	bool ble;          /* the BLE stack runs (the BT position) */
	bool p24;          /* the 2.4G link runs (the 2.4G position) */
	uint8_t active;    /* the selected profile */
	uint8_t bonded;    /* a bit per profile with a bond */
	bool connected;    /* a central is connected */
	bool ready;        /* the link is encrypted with the keyboard report's notifications on */
	bool pairing;      /* the profile has no bond: advertising discoverable */
	bool p24_linked;
	bool p24_pairing;
};
void ble_status(struct ble_status *st);
/* p24.c: the link's state for ble_status(). */
void p24_status(bool *linked, bool *pairing);

/* ble.c: the profile the stack runs; requests from ZMK's thread, taken by the BLE thread */
/* ble_running(): tlsr_ble.h */
uint8_t ble_profile_active(void);
/* The active profile, or the one a pending request selects (what ZMK's next call acts on). */
uint8_t ble_profile_selected(void);
/* select (-1: none), a mask of profiles whose bonds go, a disconnect */
void ble_profile_request(int8_t select, uint8_t clear, bool disconnect);

/* ble_att.c */
void ble_att_reset(void);
void ble_att_rx(const uint8_t *pdu, uint8_t len);
bool ble_att_ready(void);
int ble_att_notify_keyboard(const uint8_t report[8]);
int ble_att_notify_consumer(const uint8_t *report, uint8_t len);
/* The CCCD bits set now. */
uint8_t ble_att_cccd_bits(void);
/* The link is encrypted with the bond's key: its CCCD bits, unless the client wrote some on this link. */
void ble_att_bond_encrypted(uint8_t bits);
/* The BLE thread's loop: a bonded client's CCCD change held back for the rate limit goes to the bond;
 * a Service Changed indication due goes out. */
void ble_att_cccd_poll(void);
/* A hash of the attribute database: handles, types, permissions, the values that never change. */
uint32_t ble_att_db_hash(void);
void ble_att_battery_level(uint8_t level);

/* ble_battery.c: the battery level, measured in the BLE thread's loops */
void ble_battery_poll(void);

/* ble_sleep.c: deep sleep until a key, the mode switch or power in wakes the chip (a boot) */
FUNC_NORETURN void ble_deep_sleep(void);
/* ble.c: the TX power level in use */
extern uint8_t ble_tx_level;
/* ble.c: a key or the knob; advertising stopped while ZMK's USB is connected starts again */
void ble_adv_activity(void);
/* ble_zmk.c: power comes in from the cable (the charger's power-in pin) */
bool ble_cable_power(void);
/* ble_zmk.c: a USB host has the keyboard over the cable (ZMK's HID ready and power in) */
bool ble_usb_host(void);
/* ble_sleep.c: the low-power state while connected and idle; poll from the BLE thread */
void ble_low_power_poll(bool link_ready);
/* A key or knob event: the idle time starts again. */
void ble_low_power_activity(void);
/* Whether the low-power state is on (the thread may wait longer). */
bool ble_low_power_on(void);

/* ble_hid.c: the host's keyboard LED output report */
void ble_hid_leds(uint8_t leds);

/* ble_zmk.c: ZMK's BLE transport */
void ble_zmk_mode_ble(void);
void ble_zmk_flush(void);
/* ...which also carries the 2.4G link's reports: the next one ZMK queued, if any */
#define BLE_ZMK_REPORT_KEYBOARD 0U
#define BLE_ZMK_REPORT_CONSUMER 1U
bool ble_zmk_take(uint8_t *kind, uint8_t *data, uint8_t *len);
/* The 2.4G link takes ZMK's reports (a dongle is known) or not. */
void ble_zmk_p24_ready(bool ready);

/* ble.c: the radio for one exchange; flash writes wait for it (ble_flash_radio_hold) */
void ble_radio_take(void);
void ble_radio_give(void);
/* ble_conn.c: ble_conn_wait() in kernel ticks */
int ble_conn_wait_ticks(uint32_t ticks);

/*
 * Times kept in kernel ticks: the low 32 bits of k_uptime_ticks(), compared by
 * unsigned differences; a duration in milliseconds is turned into ticks when
 * compiled (users assert a whole number of ticks per millisecond).
 */
#define BLE_MS_TICKS(ms) ((uint32_t)(ms) * (uint32_t)(CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000))

/* p24.c: the proprietary 2.4G link, in the 2.4G position (CONFIG_TLSR_P24) */
FUNC_NORETURN void p24_run(const uint8_t unit[16]);
bool p24_running(void);
/* the RF interrupt while the 2.4G link runs */
void p24_rf_isr(void);
/* a key or the knob */
void p24_activity(void);
/* &p24_pair: forget the dongle and pair again */
void p24_pair_request(void);
/* CONFIG_TLSR_P24_DIAG: n octets of the counters from offset at; the number copied */
size_t p24_diag_read(size_t at, uint8_t *out, size_t n);
/* The link's counters in RAM (cidoo_p24_stats) and their size, for ble_link_stats_read() */
const uint8_t *p24_stats(size_t *len);
/* app/src/endpoints.c (zmk-tc32): re-select the endpoint */
void zmk_endpoints_reselect(void);

#endif /* TLSR_BLE_INTERNAL_H_ */

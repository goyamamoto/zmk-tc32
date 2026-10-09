/*
 * Scripted USB host for the TLSR8278 usb_dc driver test: enumerates the device
 * through the register model, then reads HID reports from the interrupt IN
 * endpoint while the application (ZMK with a mock matrix) presses keys.
 * Prints "RESULT: PASS" or "RESULT: FAIL" and exits with 0 or 1.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>
#include <zephyr/drivers/usb/usb_dc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_ch9.h>

#include "tlsr_usb_model.h"
#include "tlsr_usb_ota_host.h"

void posix_exit(int exit_code);
uint8_t zmk_hid_indicators_get_current_profile(void);

#define POLL_MS  1
#define READ_MS  3000
#define MAX_REPS 32

static int failures;

#define CHECK(cond, ...)                                                                           \
	do {                                                                                       \
		if (!(cond)) {                                                                     \
			printk("FAIL: " __VA_ARGS__);                                              \
			printk("\n");                                                              \
			failures++;                                                                \
		}                                                                                  \
	} while (0)

static void hexdump(const char *label, const uint8_t *buf, int n)
{
	printk("%s (%d):", label, n);
	for (int i = 0; i < n; i++) {
		printk(" %02x", buf[i]);
	}
	printk("\n");
}

static int get_descriptor(uint8_t type, uint8_t index, uint16_t lang, uint8_t *buf, uint16_t len)
{
	const uint8_t setup[8] = {0x80, 0x06, index, type, lang & 0xff, lang >> 8, len & 0xff,
				  len >> 8};

	return sim_control(setup, NULL, 0, buf, len);
}

#define MAX_HID 2

/* HID interfaces with an interrupt IN endpoint, in descriptor order. */
static int parse_config(const uint8_t *cfg, int len, struct tlsr_hid_info *hid)
{
	int n = 0;
	bool in_hid = false;

	for (int i = 0; i + 2 <= len && cfg[i] >= 2 && n < MAX_HID; i += cfg[i]) {
		const uint8_t *d = &cfg[i];

		if (d[1] == 0x04) { /* interface */
			in_hid = d[5] == 0x03;
			hid[n].intf = d[2];
		} else if (d[1] == 0x21 && in_hid) { /* HID */
			hid[n].report_len = d[7] | (d[8] << 8);
		} else if (d[1] == 0x05 && in_hid && (d[2] & 0x80) != 0U) {
			hid[n].in_ep = d[2] & 0x0f;
			hid[n].in_mps = d[4] | (d[5] << 8);
			n++;
			in_hid = false;
		}
	}
	return n;
}

static int set_leds(uint8_t intf, uint8_t leds)
{
	return sim_control((const uint8_t[8]){0x21, 0x09, 0x01, 0x02, intf, 0, 2, 0},
			   (const uint8_t[2]){0x01, leds}, 2, NULL, 0);
}

static void check_leds(uint8_t intf, uint8_t leds, const char *what)
{
	int n = set_leds(intf, leds);

	printk("SET_REPORT (output, %s): %d\n", what, n);
	CHECK(n == 0 && zmk_hid_indicators_get_current_profile() == leds,
	      "%s: SET_REPORT %d, HID indicators 0x%02x, expected 0x%02x", what, n,
	      zmk_hid_indicators_get_current_profile(), leds);
}

/* Bus reset and enumeration up to SET_CONFIGURATION; returns the configuration length. */
static int enumerate_again(uint8_t *buf)
{
	int n;

	sim_bus_reset();
	n = get_descriptor(0x01, 0, 0, buf, 64);
	CHECK(n == 18, "re-enumeration: device descriptor %d bytes", n);
	n = sim_control((const uint8_t[8]){0x00, 0x05, 0x06, 0, 0, 0, 0, 0}, NULL, 0, NULL, 0);
	CHECK(n == 0, "re-enumeration: SET_ADDRESS %d", n);
	n = get_descriptor(0x02, 0, 0, buf, 9);
	uint16_t total = n == 9 ? (buf[2] | (buf[3] << 8)) : 0;

	n = get_descriptor(0x02, 0, 0, buf, total);
	CHECK(n == total && total > 9, "re-enumeration: configuration %d of %u bytes", n, total);
	n = sim_control((const uint8_t[8]){0x00, 0x09, 0x01, 0, 0, 0, 0, 0}, NULL, 0, NULL, 0);
	CHECK(n == 0, "re-enumeration: SET_CONFIGURATION %d", n);
	sim_toggle_restart();
	return total;
}

/* Reads reports until @p until_ms; returns whether @p key went down and then up. */
static bool tap_seen(uint8_t ep, int64_t until_ms, uint8_t key)
{
	bool down = false;

	while (k_uptime_get() < until_ms) {
		uint8_t pkt[64];
		int n = sim_in_ep(ep, pkt, sizeof(pkt), false);

		if (n >= 0) {
			bool any = false;

			hexdump("report (poll mode)", pkt, n);
			for (int k = 1; k < n; k++) {
				any |= pkt[k] != 0U;
				down |= k >= 3 && pkt[k] == key;
			}
			if (down && !any) {
				return true;
			}
		}
		k_msleep(POLL_MS);
	}
	return false;
}

static int in_completions;

static void count_in(uint8_t ep, enum usb_dc_ep_cb_status_code status)
{
	if (status == USB_DC_EP_DATA_IN) {
		in_completions++;
	}
}

/*
 * The host takes an IN packet and no data interrupt comes (as with no interrupt
 * lines, before the driver's 1 ms poll). The next usb_dc_ep_write() must go out
 * at once instead of returning -EAGAIN until the poll, and each packet must be
 * completed exactly once. Uses the OTA interface's IN endpoint, whose HID class
 * callback does nothing (the OTA device has no int_in_ready), so the counting
 * callback put in its place changes nothing else.
 */
static void check_in_done_without_irq(uint8_t ep)
{
	uint8_t rep[33] = {5};
	uint8_t pkt[64];
	uint32_t n;
	int w1, w2, got1, got2;
	unsigned int key;

	usb_dc_ep_set_callback(USB_EP_DIR_IN | ep, count_in);
	in_completions = 0;
	key = irq_lock(); /* no interrupt, no poll */
	w1 = usb_dc_ep_write(USB_EP_DIR_IN | ep, rep, sizeof(rep), &n);
	got1 = sim_in_ep(ep, pkt, sizeof(pkt), false);
	w2 = usb_dc_ep_write(USB_EP_DIR_IN | ep, rep, sizeof(rep), &n);
	irq_unlock(key);
	got2 = sim_in_ep(ep, pkt, sizeof(pkt), false);
	k_msleep(5); /* the poll completes the second packet */
	printk("IN packet taken without an interrupt: writes %d, %d; completions %d\n", w1, w2,
	       in_completions);
	CHECK(w1 == 0 && got1 == (int)sizeof(rep), "first write %d, host got %d", w1, got1);
	CHECK(w2 == 0, "write after a packet taken without an interrupt: %d, expected 0", w2);
	CHECK(got2 == (int)sizeof(rep), "host got %d bytes of the second packet", got2);
	CHECK(in_completions == 2, "%d IN completions for 2 packets", in_completions);
}

static void host(void)
{
	static uint8_t buf[512];
	struct tlsr_hid_info hids[MAX_HID] = {0};
	struct tlsr_hid_info hid;
	int nhid;
	uint8_t reports[MAX_REPS][16];
	int report_len[MAX_REPS];
	int nreports = 0;
	int n;

	for (int i = 0; i < 1000 && !sim_dp_pullup(); i++) {
		k_msleep(1);
	}
	CHECK(sim_dp_pullup(), "the device never enabled its DP pull-up");
	sim_bus_reset();

	n = get_descriptor(0x01, 0, 0, buf, 64);
	hexdump("device descriptor", buf, n);
	CHECK(n == 18 && buf[0] == 18 && buf[1] == 0x01, "device descriptor: %d bytes", n);
	CHECK(n < 8 || buf[7] == 8, "bMaxPacketSize0 %u, expected 8", n < 8 ? 0 : buf[7]);
	if (n >= 12) {
		printk("VID:PID %04x:%04x\n", buf[8] | (buf[9] << 8), buf[10] | (buf[11] << 8));
	}

	n = sim_control((const uint8_t[8]){0x00, 0x05, 0x05, 0, 0, 0, 0, 0}, NULL, 0, NULL, 0);
	CHECK(n == 0, "SET_ADDRESS: %d", n);

	n = get_descriptor(0x02, 0, 0, buf, 9);
	CHECK(n == 9, "configuration descriptor header: %d bytes", n);
	uint16_t total = n == 9 ? (buf[2] | (buf[3] << 8)) : 0;

	n = get_descriptor(0x02, 0, 0, buf, total);
	hexdump("configuration descriptor", buf, n);
	CHECK(n == total && total > 9, "configuration descriptor: %d of %u bytes", n, total);
	nhid = parse_config(buf, n, hids);
	CHECK(nhid >= 1, "no HID interface with an IN endpoint");
	for (int i = 0; i < nhid; i++) {
		printk("HID interface %u, report descriptor %u bytes, IN endpoint %u (mps %u)\n",
		       hids[i].intf, hids[i].report_len, hids[i].in_ep, hids[i].in_mps);
	}
	hid = hids[0];

	n = get_descriptor(0x03, 2, 0x0409, buf, 255);
	CHECK(n >= 2 && n == buf[0] && buf[1] == 0x03, "product string: %d bytes", n);
	if (n >= 2) {
		char product[64] = {0};

		for (int i = 2; i + 1 < n && i / 2 < sizeof(product); i += 2) {
			product[i / 2 - 1] = buf[i];
		}
		printk("product: \"%s\"\n", product);
		CHECK(strcmp(product, CONFIG_ZMK_KEYBOARD_NAME) == 0, "product \"%s\", expected \"%s\"",
		      product, CONFIG_ZMK_KEYBOARD_NAME);
	}

	/* "ZMK Project" is 24 bytes, three full packets: the device ends with a ZLP. */
	n = get_descriptor(0x03, 1, 0x0409, buf, 255);
	CHECK(n == 24 && buf[0] == 24, "manufacturer string (ZLP): %d bytes", n);

	/* wLength ends the transfer on a full packet: no ZLP, no extra data. */
	n = get_descriptor(0x02, 0, 0, buf, 16);
	CHECK(n == 16, "configuration descriptor truncated to 16: %d bytes", n);

	/* A control read the host abandons after one packet; the next SETUP must work. */
	n = sim_control_abandon((const uint8_t[8]){0x80, 0x06, 0, 0x02, 0, 0, total & 0xff,
						   total >> 8}, 1);
	CHECK(n == 8, "abandoned configuration read: %d bytes", n);
	n = get_descriptor(0x01, 0, 0, buf, 64);
	CHECK(n == 18, "device descriptor after an abandoned transfer: %d bytes", n);

	n = sim_control((const uint8_t[8]){0x00, 0x09, 0x01, 0, 0, 0, 0, 0}, NULL, 0, NULL, 0);
	CHECK(n == 0, "SET_CONFIGURATION: %d", n);
	sim_toggle_restart();

	n = sim_control((const uint8_t[8]){0x80, 0x08, 0, 0, 0, 0, 1, 0}, NULL, 0, buf, 1);
	CHECK(n == 1 && buf[0] == 1, "GET_CONFIGURATION: %d bytes, value %u", n, buf[0]);

	n = sim_control((const uint8_t[8]){0x81, 0x06, 0, 0x22, hid.intf, 0, hid.report_len & 0xff,
					   hid.report_len >> 8},
			NULL, 0, buf, sizeof(buf));
	CHECK(n == hid.report_len, "HID report descriptor: %d of %u bytes", n, hid.report_len);

	/* The legacy HID class stalls SET_IDLE without CONFIG_USB_DEVICE_SOF. */
	n = sim_control((const uint8_t[8]){0x21, 0x0a, 0, 0, hid.intf, 0, 0, 0}, NULL, 0, NULL, 0);
	printk("SET_IDLE: %d\n", n);
	CHECK(n == 0 || n == SIM_STALL, "SET_IDLE: %d", n);

	/*
	 * Output report (keyboard LEDs, report ID 1): the OUT data stage, with
	 * the EP0 pointer holding the byte count, 0 or 8 after the packet (the
	 * hardware behavior is not known). The rest of the test keeps 0.
	 */
	sim_set_ep0_out_ptr(SIM_PTR_COUNT);
	check_leds(hid.intf, 0x02, "EP0 pointer = count");
	sim_set_ep0_out_ptr(SIM_PTR_EIGHT);
	check_leds(hid.intf, 0x01, "EP0 pointer = 8");
	sim_set_ep0_out_ptr(SIM_PTR_ZERO);
	check_leds(hid.intf, 0x04, "EP0 pointer = 0");

	/* Interrupt IN: alternate between the data interrupt and the poll fallback. */
	int64_t end = k_uptime_get() + READ_MS;
	bool use_irq = true;

	while (k_uptime_get() < end && nreports < MAX_REPS) {
		uint8_t pkt[64];

		n = sim_in_ep(hid.in_ep, pkt, sizeof(pkt), use_irq);
		if (n >= 0) {
			memcpy(reports[nreports], pkt, MIN(n, 16));
			report_len[nreports++] = n;
			hexdump(use_irq ? "report (irq)" : "report (poll)", pkt, n);
			use_irq = !use_irq;
		}
		k_msleep(POLL_MS);
	}

	/* The mock matrix taps A then B: expect A down, up, B down, up. */
	int a_down = -1, b_down = -1, up_after_a = -1, up_after_b = -1;

	for (int i = 0; i < nreports; i++) {
		bool any = false;

		for (int k = 1; k < report_len[i]; k++) {
			any |= reports[i][k] != 0U;
		}
		for (int k = 3; k < report_len[i]; k++) {
			if (reports[i][k] == 0x04 && a_down < 0) {
				a_down = i;
			}
			if (reports[i][k] == 0x05 && b_down < 0) {
				b_down = i;
			}
		}
		if (!any && a_down >= 0 && up_after_a < 0 && i > a_down) {
			up_after_a = i;
		}
		if (!any && b_down >= 0 && up_after_b < 0 && i > b_down) {
			up_after_b = i;
		}
	}
	CHECK(a_down >= 0, "no report with key A (0x04)");
	CHECK(up_after_a > a_down, "no release report after A");
	CHECK(b_down > up_after_a, "no report with key B (0x05) after A was released");
	CHECK(up_after_b > b_down, "no release report after B");

#ifdef CONFIG_TLSR_SLOTS_SIM
	CHECK(nhid == 2, "%d HID interfaces, expected the keyboard and the OTA one", nhid);
	if (nhid == 2) {
		failures += tlsr_usb_ota_host_test(&hids[1]);
		check_in_done_without_irq(hids[1].in_ep);
	}
	failures += tlsr_prev_fw_host_test();
#endif

	/*
	 * Without interrupt lines (the driver then polls):
	 * re-enumerate, set the LEDs, ask the OTA interface for its version,
	 * read the mock matrix's last tap of A (t = 19.65 s), then the update
	 * gate and the confirm with its OTA tests (several seconds, so after the
	 * tap).
	 */
	printk("-- poll mode: no USB interrupt lines\n");
	sim_set_irq_lines(false);
	enumerate_again(buf);
	check_leds(hid.intf, 0x02, "poll mode");
#ifdef CONFIG_TLSR_SLOTS_SIM
	if (nhid == 2) {
		failures += tlsr_usb_ota_host_version(&hids[1]);
	}
#endif
	CHECK(tap_seen(hid.in_ep, 22000, 0x04), "poll mode: no tap of A");
#ifdef CONFIG_TLSR_SLOTS_SIM
	if (nhid == 2) {
		failures += tlsr_usb_ota_host_gate(&hids[1]);
		failures += tlsr_usb_ota_host_confirm(&hids[1]);
		failures += tlsr_usb_ota_host_test_edges(&hids[1]);
	}
#endif

	int checked, errors;

	sim_toggle_stats(&checked, &errors);
	printk("data toggle: %d IN packets checked, %d wrong\n", checked, errors);
	CHECK(checked > 20 && errors == 0, "data toggle: %d of %d IN packets wrong", errors, checked);

	printk("RESULT: %s (%d failure%s, %d reports)\n", failures == 0 ? "PASS" : "FAIL", failures,
	       failures == 1 ? "" : "s", nreports);
	posix_exit(failures == 0 ? 0 : 1);
}

K_THREAD_DEFINE(tlsr_usb_host, 4096, host, NULL, NULL, NULL, 5, 0, 200);

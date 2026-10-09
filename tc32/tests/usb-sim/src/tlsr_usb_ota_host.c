/*
 * OTA part of the scripted USB host: replays frames made by telink_ota.py to
 * the OTA receiver of tc32/ over SET_REPORT and reads its responses from the
 * OTA interface's interrupt IN endpoint. Flash is the native_sim flash
 * simulator; the running slot and the reboot are test hooks.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <limits.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

#include <zephyr/devicetree.h>

#include "ota_vectors.h"
#include "tlsr_slots.h"
#include "tlsr_usb_model.h"
#include "tlsr_usb_ota_host.h"

#define SLOT_A      0x00000U
#define SLOT_B      0x20000U
#define REPORT_LEN  33
#define RESP_MS     1000
#define ST_OK       0
#define ST_INDEX    1
#define ST_CRC16    2
#define ST_END      4
#define ST_IMAGE    6
#define ST_SLOT     9
#define ST_VERIFY   3
#define ST_UNTESTED 10
#define ST_RUNNING  12
#define ST_LOCKED   13
#define ST_NO_WAY_BACK 14
#define ST_NOT_BOOTABLE 15

static const struct device *const flash = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
static uint32_t running = SLOT_A; /* boot ROM register 0x63e */
static int cpu_slot = -1;         /* the slot the CPU really runs, if not "running" */
static int reboots;
static int failures;

/* One read that goes wrong: the fault_nth-th 4-byte read of fault_addr has fault_mask XORed into its byte 1. */
static uint32_t fault_addr;
static int fault_nth;
static int fault_seen;
static uint8_t fault_mask;

void tlsr_slots_sim_read_done(uint32_t off, void *buf, size_t len)
{
	if (fault_nth > 0 && off == fault_addr && len == 4U && ++fault_seen == fault_nth) {
		((uint8_t *)buf)[1] ^= fault_mask;
	}
}

/*
 * A Zbit flash's supply trims (CONFIG_TLSR_USB_OTA_SIM_TRIMS), as
 * tlsr_spi_flash.c handles them: raised at an OTA start, the fields found
 * put back by the restore, which does nothing when they are not raised. On
 * hardware a reboot also puts them back; trims_raised stays set over the
 * simulated reboot after an update.
 */
static bool trims_raised;
static int trim_raises;

void tlsr_spi_flash_trim_raise(void)
{
	trims_raised = true;
	trim_raises++;
}

void tlsr_spi_flash_trim_restore(void)
{
	trims_raised = false;
}

uint32_t tlsr_slots_sim_running_slot(void)
{
	return running;
}

void tlsr_slots_sim_reboot(void)
{
	reboots++;
}

bool tlsr_slots_sim_reset_was_watchdog(void)
{
	return false;
}

void tlsr_slots_sim_watchdog(bool start)
{
	ARG_UNUSED(start);
}

int tlsr_slots_sim_step(int step)
{
	ARG_UNUSED(step);
	return 0;
}

/*
 * The retained register of the planned-reboot mark. At this run's one boot it
 * holds another image's mark (tlsr_planned_mark(0x12345678)): a reboot into
 * that image, then this one installed with no power cycle between. So the
 * version answer reports bit 4 from the receiver's own diag_info().
 */
static uint8_t retained = 0x08U;

uint8_t tlsr_slots_sim_retained_read(void)
{
	return retained;
}

void tlsr_slots_sim_retained_write(uint8_t value)
{
	retained = value;
}

uint32_t tlsr_slots_sim_sclk_hz(void)
{
	return 16000000U;
}

int tlsr_slots_sim_cpu_view(uint32_t off, uint8_t *buf, size_t len)
{
	return flash_read(flash, (cpu_slot >= 0 ? (uint32_t)cpu_slot : running) + off, buf, len);
}

#define CHECK(cond, ...)                                                                           \
	do {                                                                                       \
		if (!(cond)) {                                                                     \
			printk("FAIL: OTA: " __VA_ARGS__);                                         \
			printk("\n");                                                              \
			failures++;                                                                \
		}                                                                                  \
	} while (0)

struct resp {
	int len;
	uint16_t idx;
	uint8_t status;
	uint8_t raw[64];
};

/* One SET_REPORT with a frame, then the response from the interrupt IN endpoint. */
static struct resp send(const struct tlsr_hid_info *hid, const uint8_t frame[REPORT_LEN])
{
	const uint8_t setup[8] = {0x21, 0x09, 0x05, 0x02, hid->intf, 0, REPORT_LEN, 0};
	struct resp r = {.len = -1};
	uint8_t buf[64];
	int n;

	n = sim_control(setup, frame, REPORT_LEN, NULL, 0);
	if (n != 0) {
		r.len = n;
		return r;
	}
	for (int t = 0; t < RESP_MS; t++) {
		n = sim_in_ep(hid->in_ep, buf, sizeof(buf), (t & 1) == 0);
		if (n >= 0) {
			r.len = n;
			memcpy(r.raw, buf, MIN(n, (int)sizeof(r.raw)));
			r.idx = n > 10 ? sys_get_le16(&buf[9]) : 0;
			r.status = n > 29 ? buf[29] : 0xff;
			return r;
		}
		k_msleep(1);
	}
	return r;
}

/* A command frame (0xff00-0xff05) with no arguments. */
static void command(uint8_t frame[REPORT_LEN], uint16_t cmd)
{
	static const uint8_t version[REPORT_LEN] = {0x05, 11, 0x01, 9, 5, 0x04, 0x52, 0x28, 0x00, 0x00, 0xff};

	memcpy(frame, version, REPORT_LEN);
	sys_put_le16(cmd, &frame[9]);
}

/* Sends frames[0..count) expecting every chunk acknowledged; returns the chunks sent. */
static int send_all(const struct tlsr_hid_info *hid, const uint8_t (*frames)[REPORT_LEN],
		    int count, const char *name)
{
	for (int i = 0; i < count; i++) {
		struct resp r = send(hid, frames[i]);
		uint16_t idx = sys_get_le16(&frames[i][9]);
		/* Chunks: the next index. START and the test start: 0, as the protocol answers START. */
		uint16_t want = idx < 0xff00U ? idx + 1U : (idx == 0xff01U || idx == 0xff04U) ? 0U : idx;

		if (r.len != REPORT_LEN || r.idx != want || r.status != ST_OK) {
			CHECK(false, "%s frame %d (index 0x%04x): response %d bytes, index 0x%04x, status %u",
			      name, i, idx, r.len, r.idx, r.status);
			return i;
		}
	}
	return count;
}

static bool flash_equals(uint32_t addr, const uint8_t *data, size_t len)
{
	uint8_t buf[64];

	for (size_t off = 0; off < len; off += sizeof(buf)) {
		size_t n = MIN(sizeof(buf), len - off);

		if (flash_read(flash, addr + off, buf, n) != 0 || memcmp(buf, &data[off], n) != 0) {
			printk("flash differs at 0x%05x\n", (unsigned int)(addr + off));
			return false;
		}
	}
	return true;
}

static uint32_t flag_word(uint32_t slot)
{
	uint32_t v = 0;

	(void)flash_read(flash, slot + 8U, &v, sizeof(v));
	return v;
}

/*
 * Whether the planned-reboot mark (the retained register tlsr_slots.c
 * simulates) names the image whose OTA file is img: its tag is the file's
 * last four bytes, the CRC-32 word.
 */
static bool marked_for(const uint8_t *img, size_t len)
{
	return tlsr_slots_sim_retained_read() == tlsr_planned_mark(sys_get_le32(&img[len - 4U]));
}

static void wait_reboot(int want)
{
	for (int t = 0; t < 500 && reboots < want; t++) {
		k_msleep(1);
	}
}

int tlsr_usb_ota_host_test(const struct tlsr_hid_info *hid)
{
	const int start = failures;
	static const uint8_t knlt_header[16] = {0x0e, 0x80, 0, 0, 0, 0, 0, 0, 'K', 'N', 'L', 'T'};
	const int good_n = ARRAY_SIZE(ota_frames_good);
	uint8_t frame[REPORT_LEN];
	struct resp r;

	CHECK(device_is_ready(flash), "flash simulator not ready");
	/* The image running from slot A. */
	CHECK(flash_erase(flash, SLOT_A, 4096) == 0 && flash_write(flash, SLOT_A, knlt_header, 16) == 0,
	      "prepare slot A");

	/* 0. Register 0x63e says B, the CPU runs A: chunk 0 is refused, nothing erased. */
	running = SLOT_B;
	cpu_slot = SLOT_A;
	CHECK(send_all(hid, ota_frames_good, 1, "good") == 1, "start (wrong slot)");
	r = send(hid, ota_frames_good[1]);
	CHECK(r.idx == 0U && r.status == ST_SLOT, "chunk 0 with a wrong running slot: index %u, status %u",
	      r.idx, r.status);
	CHECK(flag_word(SLOT_B) == 0xffffffffU, "slot B written with a wrong running slot");
	cpu_slot = -1;

	/* 1. From slot A into slot B, with a corrupted chunk and a repeated one on the way. */
	running = SLOT_A;
	CHECK(send_all(hid, ota_frames_good, 4, "good") == 4, "start and chunks 0-2");
	memcpy(frame, ota_frames_good[4], REPORT_LEN); /* chunk 3 */
	frame[20] ^= 0x40U;
	r = send(hid, frame);
	CHECK(r.idx == 3U && r.status == ST_CRC16, "corrupted chunk 3: index %u, status %u", r.idx,
	      r.status);
	CHECK(send_all(hid, &ota_frames_good[4], 3, "good") == 3, "chunks 3-5");
	r = send(hid, ota_frames_good[6]); /* chunk 5 again */
	CHECK(r.idx == 6U && r.status == ST_OK, "repeated chunk 5: index %u, status %u", r.idx,
	      r.status);
	CHECK(send_all(hid, &ota_frames_good[7], good_n - 8, "good") == good_n - 8, "chunks 6-%u",
	      OTA_LAST_GOOD);
	CHECK((flag_word(SLOT_B) & 0xffU) == 0xffU &&
		      flash_equals(SLOT_B + 9U, &ota_img_good[9], sizeof(ota_img_good) - 9U),
	      "slot B before the end command: flag %02x", flag_word(SLOT_B) & 0xffU);
	CHECK(reboots == 0, "rebooted before the end command");
	CHECK(send_all(hid, &ota_frames_good[good_n - 1], 1, "good") == 1, "end command");
	wait_reboot(1);
	CHECK(reboots == 1, "no reboot after the end command");
	CHECK(marked_for(ota_img_good, sizeof(ota_img_good)),
	      "the reboot after the update is not marked for image 1: 0x%02x",
	      tlsr_slots_sim_retained_read());
	CHECK(flash_equals(SLOT_B, ota_img_good, sizeof(ota_img_good)), "slot B holds the image");
	CHECK(flag_word(SLOT_A) == 0U, "slot A flag word 0x%08x, expected 0", flag_word(SLOT_A));
	/* Raised at the start and left so for the reboot, which puts them back on hardware. */
	CHECK(trims_raised && trim_raises > 0, "update: trims raised %d (%d raises)", trims_raised, trim_raises);
	trims_raised = false;
	printk("OTA A->B: %u chunks, slot B flag %02x\n", OTA_LAST_GOOD + 1U, flag_word(SLOT_B) & 0xffU);

	/* 2. From slot B, an image with a wrong CRC-32: refused, slot B stays bootable. */
	running = SLOT_B;
	const int bad_n = ARRAY_SIZE(ota_frames_bad_crc);

	CHECK(send_all(hid, ota_frames_bad_crc, bad_n - 2, "bad_crc") == bad_n - 2,
	      "bad_crc chunks before the last");
	r = send(hid, ota_frames_bad_crc[bad_n - 2]);
	CHECK(r.idx == OTA_LAST_BAD_CRC && r.status == ST_IMAGE,
	      "bad CRC-32 chunk: index %u, status %u", r.idx, r.status);
	CHECK(!trims_raised, "bad CRC-32: the trims stay raised after the aborted update");
	r = send(hid, ota_frames_bad_crc[bad_n - 1]);
	CHECK(r.status == ST_END, "end after bad CRC-32: status %u", r.status);
	k_msleep(100);
	CHECK(reboots == 1, "rebooted after a refused image");
	CHECK((flag_word(SLOT_B) & 0xffU) == 0x4bU, "slot B flag %02x after a refused image",
	      flag_word(SLOT_B) & 0xffU);
	CHECK((flag_word(SLOT_A) & 0xffU) != 0x4bU, "slot A bootable after a refused image");

	/* 3. A chunk out of order aborts; later chunks are refused until chunk 0. */
	CHECK(send_all(hid, ota_frames_good, 2, "good") == 2, "start and chunk 0");
	r = send(hid, ota_frames_good[3]); /* chunk 2 */
	CHECK(r.idx == 2U && r.status == ST_INDEX, "chunk 2 after 0: index %u, status %u", r.idx,
	      r.status);
	CHECK(!trims_raised, "chunk out of order: the trims stay raised after the aborted update");
	r = send(hid, ota_frames_good[2]); /* chunk 1 */
	CHECK(r.idx == 1U && r.status == ST_INDEX, "chunk 1 after abort: index %u, status %u", r.idx,
	      r.status);

	/* 4. From slot B into slot A. */
	CHECK(send_all(hid, ota_frames_good2, ARRAY_SIZE(ota_frames_good2), "good2") ==
		      ARRAY_SIZE(ota_frames_good2), "good2 image");
	wait_reboot(2);
	CHECK(reboots == 2, "no reboot after the second image");
	CHECK(marked_for(ota_img_good2, sizeof(ota_img_good2)),
	      "the reboot after the update is not marked for image 2: 0x%02x",
	      tlsr_slots_sim_retained_read());
	CHECK(flash_equals(SLOT_A, ota_img_good2, sizeof(ota_img_good2)), "slot A holds image 2");
	CHECK(flag_word(SLOT_B) == 0U, "slot B flag word 0x%08x, expected 0", flag_word(SLOT_B));
	printk("OTA B->A: %u chunks, slot A flag %02x\n", OTA_LAST_GOOD2 + 1U,
	       flag_word(SLOT_A) & 0xffU);

	return failures - start;
}

static void wait_until(int64_t ms)
{
	while (k_uptime_get() < ms) {
		k_msleep(10);
	}
}

int tlsr_prev_fw_host_test(void)
{
	const int start = failures;
	const int before = reboots;

	CHECK(k_uptime_get() < 11000, "OTA tests ran into the key events (%lld ms)",
	      k_uptime_get());
	running = SLOT_A; /* image 2, installed by the last OTA test */
	const uint32_t gen = tlsr_slot_generation();

	wait_until(12600);
	CHECK(reboots == before, "&prev_fw held 1 s went back");
	CHECK(flag_word(SLOT_B) == 0U, "slot B flag word 0x%08x after 1 s", flag_word(SLOT_B));
	wait_until(17000);
	CHECK(reboots == before + 1, "&prev_fw held 3.5 s: %d reboots", reboots - before);
	CHECK(marked_for(ota_img_good, sizeof(ota_img_good)),
	      "the reboot of &prev_fw is not marked for image 1 in slot B: 0x%02x",
	      tlsr_slots_sim_retained_read());
	CHECK(flag_word(SLOT_B) == 0x544c4e4bU, "slot B flag word 0x%08x", flag_word(SLOT_B));
	CHECK(flag_word(SLOT_A) == 0U, "slot A flag word 0x%08x", flag_word(SLOT_A));
	CHECK(flash_equals(SLOT_B + 16U, &ota_img_good[16], sizeof(ota_img_good) - 16U),
	      "slot B image changed");
	/* The revert returned (the reboot is a test hook): the version reply checks slot B again. */
	CHECK(tlsr_slot_generation() != gen, "&prev_fw: the slot generation did not change");
	printk("&prev_fw: slot B boots next\n");
	return failures - start;
}

int tlsr_usb_ota_host_version(const struct tlsr_hid_info *hid)
{
	const uint8_t frame[REPORT_LEN] = {0x05, 11, 0x01, 9, 5, 0x04, 0x52, 0x28, 0x00, 0x00, 0xff};
	const int start = failures;
	struct resp r = send(hid, frame);

	CHECK(r.len == REPORT_LEN && r.idx == 0xff00U && r.status == ST_OK,
	      "version command: %d bytes, index 0x%04x, status %u", r.len, r.idx, r.status);
	/*
	 * Clock, watchdog capture, boots counted (this first boot is 1: every
	 * unplanned boot counts until a healthy one, and the host has not
	 * configured USB when the guard boots), flags (measured, guard), "ZC".
	 */
	CHECK(sys_get_le32(&r.raw[11]) == 16000000U && sys_get_le16(&r.raw[15]) == 244U &&
		      r.raw[17] == 1U && r.raw[18] == 3U && r.raw[21] == 'Z' && r.raw[22] == 'C',
	      "version info: clock %u, capture %u, boots %u, flags %u, mark %c%c",
	      sys_get_le32(&r.raw[11]), sys_get_le16(&r.raw[15]), r.raw[17], r.raw[18], r.raw[21],
	      r.raw[22]);
	printk("version info: clock %u Hz, watchdog capture %u\n", sys_get_le32(&r.raw[11]),
	       sys_get_le16(&r.raw[15]));
	/*
	 * The flash status bytes (19-20 as found, 23-24 as left, 25 the flags,
	 * 26-28 the JEDEC ID) against what the guard itself reports, byte for
	 * byte: a scramble of the packing fails here. On native_sim the unlock is
	 * not built (no SPI flash driver), so the guard reports nothing read: all 0.
	 */
	{
		uint16_t st_boot;
		uint16_t st_now;
		uint8_t flags;
		uint32_t mid;

		tlsr_boot_guard_flash_info(&st_boot, &st_now, &flags, &mid);
		CHECK(sys_get_le16(&r.raw[19]) == st_boot && sys_get_le16(&r.raw[23]) == st_now &&
			      r.raw[25] == flags && r.raw[26] == (uint8_t)mid &&
			      r.raw[27] == (uint8_t)(mid >> 8) && r.raw[28] == (uint8_t)(mid >> 16),
		      "flash bytes: status 0x%04x/0x%04x flags 0x%02x id %02x%02x%02x, guard says 0x%04x/0x%04x 0x%02x 0x%06x",
		      sys_get_le16(&r.raw[19]), sys_get_le16(&r.raw[23]), r.raw[25], r.raw[28], r.raw[27],
		      r.raw[26], st_boot, st_now, flags, mid);
		CHECK((flags & BIT(4)) == 0U && st_boot == 0U && st_now == 0U && mid == 0U,
		      "no SPI flash driver here: the guard reports the register not read (flags 0x%02x)", flags);
	}
	/*
	 * With non-zero values (as after an unlock of a locked GD25LD40C): each
	 * field in its own place, so a permutation of the packing fails here.
	 */
	tlsr_boot_guard_sim_flash(0x0018U, 0x0000U, 1, 0x1360c8U);
	r = send(hid, frame);
	CHECK(r.len == REPORT_LEN && sys_get_le16(&r.raw[19]) == 0x0018U && sys_get_le16(&r.raw[23]) == 0x0000U &&
		      r.raw[25] == (BIT(0) | BIT(4)) && r.raw[26] == 0xc8U && r.raw[27] == 0x60U && r.raw[28] == 0x13U,
	      "flash bytes with values: status 0x%04x/0x%04x flags 0x%02x id %02x%02x%02x, want 0x0018/0x0000 0x11 1360c8",
	      sys_get_le16(&r.raw[19]), sys_get_le16(&r.raw[23]), r.raw[25], r.raw[28], r.raw[27], r.raw[26]);
	tlsr_boot_guard_sim_flash(0U, 0U, INT_MIN, 0U);
	/*
	 * The diagnostics (30-32) against the slots code and the clock: present,
	 * the other slot's check, the running slot, no CPU figure unless asked
	 * (and none on native_sim, whose clock does not run in a busy loop even
	 * when asked: request byte 32 = 1), the uptime in seconds.
	 */
	for (int ask = 0; ask < 2; ask++) {
		uint8_t req[REPORT_LEN];

		memcpy(req, frame, sizeof(req));
		req[32] = (uint8_t)ask;
		r = send(hid, req);

		const bool other_ok = tlsr_slot_check(tlsr_slot_other(), NULL) == 0;
		const uint32_t up = (uint32_t)MIN(k_uptime_get() / 1000, 255);

		CHECK(r.len == REPORT_LEN && r.status == ST_OK && (r.raw[30] & BIT(6)) != 0U &&
			      ((r.raw[30] & BIT(0)) != 0U) == other_ok &&
			      ((r.raw[30] & BIT(7)) != 0U) == (running == SLOT_B) && r.raw[31] == 0xffU &&
			      r.raw[32] <= up && r.raw[32] + 1U >= up,
		      "diagnostics (asked %d): flags 0x%02x, cpu 0x%02x, uptime %u s (other slot %s, uptime %u s)",
		      ask, r.raw[30], r.raw[31], r.raw[32], other_ok ? "checks" : "does not check", up);
		/*
		 * The planned-reboot mark: reported (bit 5); this run's one boot
		 * found another image's mark (bit 4), so it was not planned (bit 3).
		 */
		CHECK((r.raw[30] & (BIT(3) | BIT(4) | BIT(5))) == (BIT(4) | BIT(5)),
		      "diagnostics (asked %d): mark bits 0x%02x, want bits 4 and 5", ask, r.raw[30] & 0x38U);
		CHECK((r.raw[18] & BIT(4)) == 0U, "diagnostics (asked %d): byte 18 0x%02x says the mark was stuck",
		      ask, r.raw[18]);
		printk("diagnostics (asked %d): flags 0x%02x, cpu 0x%02x, uptime %u s\n", ask, r.raw[30],
		       r.raw[31], r.raw[32]);
	}
	/*
	 * A chunk 0 with no start command begins an update of the other slot too:
	 * the next version reply checks that slot again instead of answering from
	 * the check made before. Slot B runs (&prev_fw made it the
	 * bootable one), so the update goes into slot A, whose first sector is
	 * kept here and put back; a start command then drops the unfinished update.
	 * The reply before the chunk must say the other slot checks (the check
	 * made then is what a missing reset would keep); the one made after it
	 * stays in the receiver when slot A is put back, and no later case reads
	 * byte 30. Slot A's image checks, so the chunk needs the unlock (the gate).
	 */
	{
		static uint8_t keep[4096];
		const uint32_t was = running;
		uint8_t unlock[REPORT_LEN];

		running = SLOT_B;
		CHECK(flash_read(flash, SLOT_A, keep, sizeof(keep)) == 0 &&
			      tlsr_slot_check(SLOT_A, NULL) == 0,
		      "slot A holds an image that checks before the bare chunk 0");
		r = send(hid, frame);
		CHECK(r.len == REPORT_LEN && (r.raw[30] & (BIT(0) | BIT(6))) == (BIT(0) | BIT(6)),
		      "before the bare chunk 0: flags 0x%02x, want the other slot checking", r.raw[30]);
		command(unlock, 0xff05U);
		r = send(hid, unlock);
		CHECK(r.idx == 0xff05U && r.status == ST_OK, "unlock before the bare chunk 0: index 0x%04x, status %u",
		      r.idx, r.status);
		r = send(hid, ota_frames_good[1]); /* chunk 0, no start command */
		CHECK(r.idx == 1U && r.status == ST_OK, "bare chunk 0: index %u, status %u", r.idx,
		      r.status);
		r = send(hid, frame);
		CHECK(r.len == REPORT_LEN &&
			      (r.raw[30] & (BIT(0) | BIT(6) | BIT(7))) == (BIT(6) | BIT(7)) &&
			      tlsr_slot_check(SLOT_A, NULL) != 0,
		      "after a bare chunk 0: flags 0x%02x, want slot B running and slot A not checking",
		      r.raw[30]);
		printk("diagnostics after a bare chunk 0: flags 0x%02x\n", r.raw[30]);
		CHECK(flash_erase(flash, SLOT_A, sizeof(keep)) == 0 &&
			      flash_write(flash, SLOT_A, keep, sizeof(keep)) == 0 &&
			      tlsr_slot_check(SLOT_A, NULL) == 0,
		      "slot A put back");
		r = send(hid, ota_frames_good[0]); /* start: the unfinished update is dropped */
		CHECK(r.idx == 0U && r.status == ST_OK, "start after the bare chunk 0: index %u, status %u",
		      r.idx, r.status);
		running = was;
	}
	return failures - start;
}

/*
 * The gate (CONFIG_TLSR_USB_OTA_GATE; CONFIG_TLSR_USB_OTA_ARM_MS is 300 here).
 * Slot B runs; slot A holds image 2, which checks (&prev_fw cleared its flag
 * word). Its first sector is kept and put back after the update case 3 begins.
 * 1. the version reply shows the gate, not unlocked;
 * 2. chunk 0 after a start command, and a bare one: status 13 at index 0, no
 *    update under way after it, slot A unchanged;
 * 3. the unlock: answered with the version information, unlocked shown;
 *    chunk 0 is then taken, and the unlock is used up: the reply after it no
 *    longer shows it, and with slot A put back the next chunk 0 gets status 13;
 * 4. an unlock whose window has passed: status 13, slot A unchanged;
 * 5. an OTA test after the unlock does not use it up: the version answer
 *    still shows it, and chunk 0 of an update over slot A's image is then
 *    taken (slot A put back after it).
 * A start command at the end drops the update begun in 5. Without an unlock:
 * the OTA test (tlsr_usb_ota_host_confirm, with slot A checking) and the
 * updates into a slot where nothing checks (1-4 of tlsr_usb_ota_host_test).
 */
int tlsr_usb_ota_host_gate(const struct tlsr_hid_info *hid)
{
	static uint8_t keep[4096];
	const int start = failures;
	uint8_t version[REPORT_LEN];
	uint8_t frame[REPORT_LEN];
	struct resp r;

	running = SLOT_B;
	command(version, 0xff00U);
	CHECK(flash_read(flash, SLOT_A, keep, sizeof(keep)) == 0 && tlsr_slot_check(SLOT_A, NULL) == 0,
	      "gate: slot A holds an image that checks");

	/* 1. */
	r = send(hid, version);
	CHECK(r.len == REPORT_LEN && (r.raw[30] & (BIT(1) | BIT(2) | BIT(6))) == (BIT(2) | BIT(6)),
	      "gate: version flags 0x%02x, want the gate shown, not unlocked", r.raw[30]);

	/* 2. */
	command(frame, 0xff01U);
	r = send(hid, frame);
	CHECK(r.idx == 0U && r.status == ST_OK, "gate: start: index 0x%04x, status %u", r.idx, r.status);
	for (int bare = 0; bare < 2; bare++) {
		r = send(hid, ota_frames_good[1]);
		CHECK(r.idx == 0U && r.status == ST_LOCKED,
		      "gate: chunk 0 %s, not unlocked: index %u, status %u",
		      bare ? "bare" : "after a start", r.idx, r.status);
	}
	r = send(hid, ota_frames_good[2]);
	CHECK(r.status == ST_INDEX, "gate: chunk 1 after a refused chunk 0: status %u", r.status);
	CHECK(flash_equals(SLOT_A, keep, sizeof(keep)) && tlsr_slot_check(SLOT_A, NULL) == 0,
	      "gate: slot A changed by a refused update");

	/* 3. */
	command(frame, 0xff05U);
	r = send(hid, frame);
	CHECK(r.len == REPORT_LEN && r.idx == 0xff05U && r.status == ST_OK && r.raw[21] == 'Z' &&
		      r.raw[22] == 'C' &&
		      (r.raw[30] & (BIT(1) | BIT(2) | BIT(6))) == (BIT(1) | BIT(2) | BIT(6)),
	      "gate: unlock: index 0x%04x, status %u, mark %c%c, flags 0x%02x", r.idx, r.status,
	      r.raw[21], r.raw[22], r.raw[30]);
	r = send(hid, version);
	CHECK((r.raw[30] & BIT(1)) != 0U, "gate: version after the unlock: flags 0x%02x", r.raw[30]);
	r = send(hid, ota_frames_good[1]);
	CHECK(r.idx == 1U && r.status == ST_OK, "gate: chunk 0 unlocked: index %u, status %u", r.idx,
	      r.status);
	r = send(hid, version);
	CHECK((r.raw[30] & BIT(1)) == 0U, "gate: version after chunk 0: flags 0x%02x, want the unlock used up",
	      r.raw[30]);
	CHECK(flash_erase(flash, SLOT_A, sizeof(keep)) == 0 &&
		      flash_write(flash, SLOT_A, keep, sizeof(keep)) == 0 &&
		      tlsr_slot_check(SLOT_A, NULL) == 0,
	      "gate: slot A put back");
	r = send(hid, ota_frames_good[1]);
	CHECK(r.idx == 0U && r.status == ST_LOCKED,
	      "gate: chunk 0 again, the unlock used up: index %u, status %u", r.idx, r.status);

	/* 4. */
	command(frame, 0xff05U);
	r = send(hid, frame);
	CHECK(r.idx == 0xff05U && r.status == ST_OK, "gate: second unlock: index 0x%04x, status %u", r.idx,
	      r.status);
	k_msleep(CONFIG_TLSR_USB_OTA_ARM_MS + 100);
	r = send(hid, version);
	CHECK((r.raw[30] & BIT(1)) == 0U, "gate: version after the window: flags 0x%02x", r.raw[30]);
	r = send(hid, ota_frames_good[1]);
	CHECK(r.idx == 0U && r.status == ST_LOCKED, "gate: chunk 0 after the window: index %u, status %u",
	      r.idx, r.status);
	CHECK(!trims_raised, "gate: the trims stay raised after a refused chunk 0");
	CHECK(flash_equals(SLOT_A, keep, sizeof(keep)) && tlsr_slot_check(SLOT_A, NULL) == 0,
	      "gate: slot A changed after the window");

	/* 5. */
	command(frame, 0xff05U);
	r = send(hid, frame);
	CHECK(r.idx == 0xff05U && r.status == ST_OK, "gate: third unlock: index 0x%04x, status %u", r.idx,
	      r.status);
	command(frame, 0xff07U);
	r = send(hid, frame);
	CHECK(r.idx == 0xff07U && r.status == ST_OK, "gate: flash test after the unlock: status %u", r.status);
	r = send(hid, version);
	CHECK((r.raw[30] & BIT(1)) != 0U, "gate: version after a flash test: flags 0x%02x, want unlocked",
	      r.raw[30]);
	CHECK(send_all(hid, ota_frames_good, 2, "good") == 2, "gate: start and chunk 0 over slot A after a test");
	CHECK(flash_erase(flash, SLOT_A, sizeof(keep)) == 0 &&
		      flash_write(flash, SLOT_A, keep, sizeof(keep)) == 0 &&
		      tlsr_slot_check(SLOT_A, NULL) == 0,
	      "gate: slot A put back after 5");

	/*
	 * 6. The version reply's check of the other slot follows its writes:
	 * image 2 sent again into slot A (unlocked), a version
	 * request after chunk 2 (slot A does not check), the rest up to the
	 * CRC-32 chunk with no end command, a version request: slot A checks
	 * again. Without the recheck the reply keeps the answer made in the
	 * middle. Slot A's flag word is then cleared, as &prev_fw left it.
	 */
	{
		const int n = ARRAY_SIZE(ota_frames_good2);
		static const uint8_t zero[4] = {0};

		command(frame, 0xff05U);
		r = send(hid, frame);
		CHECK(r.idx == 0xff05U && r.status == ST_OK, "gate: fourth unlock: status %u", r.status);
		CHECK(send_all(hid, ota_frames_good2, 4, "good2") == 4, "gate: start and chunks 0-2 into slot A");
		r = send(hid, version);
		CHECK(r.len == REPORT_LEN && (r.raw[30] & BIT(0)) == 0U,
		      "gate: version in the middle of an update: flags 0x%02x, want slot A not checking",
		      r.raw[30]);
		CHECK(send_all(hid, &ota_frames_good2[4], n - 5, "good2") == n - 5,
		      "gate: the rest of image 2, no end command");
		r = send(hid, version);
		CHECK(r.len == REPORT_LEN && (r.raw[30] & BIT(0)) != 0U &&
			      tlsr_slot_check(SLOT_A, NULL) == 0,
		      "gate: version after the last chunk: flags 0x%02x, want slot A checking", r.raw[30]);
		CHECK(flash_write(flash, SLOT_A + 8U, zero, sizeof(zero)) == 0 &&
			      flash_equals(SLOT_A + 16U, &ota_img_good2[16], sizeof(ota_img_good2) - 16U) &&
			      flag_word(SLOT_A) == 0U,
		      "gate: slot A holds image 2 again, flag word 0x%08x", flag_word(SLOT_A));
		printk("gate: the version reply's check of slot A followed an update that stopped before its end\n");
	}

	/*
	 * 7. A refusal answers from the gate's own check: an update
	 * into slot A begun (unlocked) and a version request after chunk 2 (not
	 * checking), then image 2 put back behind the receiver's back (no write
	 * of its own, so the generation stays) and chunk 0 again: refused, and
	 * the next version reply says slot A checks. Without the gate's answer
	 * it keeps the one made in the middle.
	 */
	{
		static const uint8_t zero[4] = {0};

		command(frame, 0xff05U);
		r = send(hid, frame);
		CHECK(r.idx == 0xff05U && r.status == ST_OK, "gate: fifth unlock: status %u", r.status);
		CHECK(send_all(hid, ota_frames_good2, 4, "good2") == 4, "gate: 7: start and chunks 0-2 into slot A");
		r = send(hid, version);
		CHECK(r.len == REPORT_LEN && (r.raw[30] & BIT(0)) == 0U,
		      "gate: 7: version in the middle: flags 0x%02x, want slot A not checking", r.raw[30]);
		CHECK(flash_erase(flash, SLOT_A, ROUND_UP(sizeof(ota_img_good2), 4096U)) == 0 &&
			      flash_write(flash, SLOT_A, ota_img_good2, sizeof(ota_img_good2)) == 0 &&
			      flash_write(flash, SLOT_A + 8U, zero, sizeof(zero)) == 0 &&
			      tlsr_slot_check(SLOT_A, NULL) == 0,
		      "gate: 7: image 2 put back in slot A");
		r = send(hid, ota_frames_good2[1]);
		CHECK(r.idx == 0U && r.status == ST_LOCKED, "gate: 7: chunk 0 over image 2: index %u, status %u",
		      r.idx, r.status);
		r = send(hid, version);
		CHECK(r.len == REPORT_LEN && (r.raw[30] & BIT(0)) != 0U,
		      "gate: 7: version after the refusal: flags 0x%02x, want slot A checking", r.raw[30]);
	}

	command(frame, 0xff01U);
	r = send(hid, frame);
	CHECK(r.idx == 0U && r.status == ST_OK, "gate: start at the end: index 0x%04x, status %u", r.idx,
	      r.status);
	printk("gate: an update over slot A's image refused (status 13) until the unlock, which one chunk 0 "
	       "uses up and which ends after %d ms\n", CONFIG_TLSR_USB_OTA_ARM_MS);
	return failures - start;
}

static bool flash_blank(uint32_t addr, size_t len)
{
	uint8_t buf[64];

	for (size_t off = 0; off < len; off += sizeof(buf)) {
		size_t n = MIN(sizeof(buf), len - off);

		if (flash_read(flash, addr + off, buf, n) != 0) {
			return false;
		}
		for (size_t i = 0; i < n; i++) {
			if (buf[i] != 0xffU) {
				printk("flash not blank at 0x%05x\n", (unsigned int)(addr + off + i));
				return false;
			}
		}
	}
	return true;
}

/* The bond log's two sectors (src/ble/ble_bond.c): the storage partition's first two. */
#define STORE_BASE   DT_REG_ADDR(DT_NODELABEL(storage_partition))
#define STORE_SECTOR 4096U
#define BOND_MAGIC   0x444e4f42U /* "BOND" */

/* The flash test's reads (ble_bond.c store_read_back()): the bond_fault-th one gets a byte flipped. */
static int bond_reads;
static int bond_fault;

void ble_bond_sim_read_done(uint32_t addr, uint8_t *buf, size_t len)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(len);
	if (++bond_reads == bond_fault) {
		buf[0] ^= 0x10U;
	}
}

/* The sequence number of a bond log sector's header, 0 when it has none. */
static uint32_t bond_seq(uint32_t sec)
{
	uint8_t hdr[8];

	if (flash_read(flash, sec, hdr, sizeof(hdr)) != 0 || sys_get_le32(&hdr[0]) != BOND_MAGIC ||
	    sys_get_le32(&hdr[4]) == 0xffffffffU) {
		return 0U;
	}
	return sys_get_le32(&hdr[4]);
}

/* A bond log sector in use: its header, one state record (magic 0xb1, no bonds), the rest blank. */
static bool bond_sector_ok(uint32_t sec, uint32_t seq)
{
	uint8_t r[64];

	if (bond_seq(sec) != seq || flash_read(flash, sec + 8U, r, sizeof(r)) != 0 || r[0] != 0xb1U ||
	    sys_get_le16(&r[62]) != crc16_ccitt(0xffff, r, 62)) {
		return false;
	}
	return flash_blank(sec + 8U + 64U, STORE_SECTOR - 8U - 64U);
}

/*
 * The confirm command needs a passed flash test in the same boot: refused
 * before any test and after a failed one. The flash test (0xff07) reads,
 * erases, writes and reads back the bond log's two sectors, nothing else:
 * both slots stay as they are, it does not reboot, the Zbit trims are
 * raised for it and put back. The confirm also needs the other slot's
 * image to check when it arrives: refused with slot A changed behind the
 * receiver's back, nothing written. With slot A put back, the test passes
 * again (the sectors swap back) and the confirm is taken.
 */
int tlsr_usb_ota_host_confirm(const struct tlsr_hid_info *hid)
{
	const uint8_t version[REPORT_LEN] = {0x05, 11, 0x01, 9, 5, 0x04, 0x52, 0x28, 0x00, 0x00, 0xff};
	const int reboots_before = reboots;
	uint8_t confirm[REPORT_LEN];
	uint8_t test[REPORT_LEN];
	const int start = failures;
	struct resp r;
	int raises;
	/*
	 * The gate test ran one flash test already (its pass does not survive
	 * the start commands since): the second sector is in use with sequence
	 * 1, the first blank. Each passed test moves the log to the other
	 * sector with the next sequence number: 64 reads of the blank check,
	 * the state record's and the header's read back, 64 of the old
	 * sector's blank check (the headers it reads first are not counted).
	 */
	uint32_t in_use = STORE_BASE + STORE_SECTOR;
	uint32_t other = STORE_BASE;
	uint32_t seq = 1U;
	const int reads_per_test = 2 * STORE_SECTOR / 64 + 2;

	/* Slot B runs image 1 (tlsr_prev_fw_host_test made it the bootable slot). */
	running = SLOT_B;
	memcpy(confirm, version, sizeof(confirm));
	confirm[9] = 0x03; /* 0xff03 */
	command(test, 0xff07U);
	r = send(hid, version);
	CHECK(r.len == REPORT_LEN && r.raw[17] == 1U, "before the confirm: boots counted %u, want 1",
	      r.raw[17]);
	CHECK(r.len == REPORT_LEN && r.raw[18] == 3U, "before the confirm: flags %u, want 3 (unconfirmed, untested)",
	      r.raw[18]);
	CHECK(bond_sector_ok(in_use, seq) && flash_blank(other, STORE_SECTOR),
	      "before the confirm: the bond log's sequence %u at the second sector (want 1), %u at the first "
	      "(want 0, blank)", bond_seq(in_use), bond_seq(other));

	/* 1. No flash test yet: the confirm is refused and changes nothing. */
	r = send(hid, confirm);
	CHECK(r.len == REPORT_LEN && r.idx == 0xff03U && r.status == ST_UNTESTED && r.raw[17] == 1U &&
		      r.raw[18] == 3U,
	      "confirm before a test: status %u, boots counted %u, flags %u", r.status, r.raw[17],
	      r.raw[18]);

	/* 2. A test whose read back of the erased sector differs fails (status 3); the confirm is still refused. */
	bond_reads = 0;
	bond_fault = 1; /* the first read of the blank check */
	raises = trim_raises;
	r = send(hid, test);
	bond_fault = 0;
	CHECK(r.len == REPORT_LEN && r.idx == 0xff07U && r.status == ST_VERIFY && r.raw[18] == 3U &&
		      r.raw[21] == 'Z' && r.raw[22] == 'C',
	      "test with the erased sector reading back wrong: index 0x%04x, status %u, flags %u", r.idx,
	      r.status, r.raw[18]);
	CHECK(bond_reads == 1, "the failed test made %d reads, want 1", bond_reads);
	CHECK(bond_sector_ok(in_use, seq) && flash_blank(other, STORE_SECTOR),
	      "after a failed blank check: the log changed (sequence %u / %u)", bond_seq(in_use), bond_seq(other));
	CHECK(!trims_raised && trim_raises == raises + 1, "a failed test: trims raised %d, %d raises (want %d)",
	      trims_raised, trim_raises, raises + 1);
	r = send(hid, confirm);
	CHECK(r.status == ST_UNTESTED && r.raw[18] == 3U, "confirm after a failed test: status %u, flags %u",
	      r.status, r.raw[18]);

	/* 2a. A test whose read back of a written record differs fails the same way (the header stays in use). */
	bond_reads = 0;
	bond_fault = STORE_SECTOR / 64 + 1; /* the blank check's 64 reads, then the state record's */
	r = send(hid, test);
	bond_fault = 0;
	CHECK(r.status == ST_VERIFY && r.raw[18] == 3U,
	      "test with a record reading back wrong: status %u, flags %u (%d reads)", r.status, r.raw[18],
	      bond_reads);
	CHECK(bond_reads == STORE_SECTOR / 64 + 1, "the test stopped after %d reads, want %u", bond_reads,
	      STORE_SECTOR / 64 + 1);
	CHECK(bond_sector_ok(in_use, seq) && bond_seq(other) == 0U,
	      "after a failed record read back: the sector in use changed (sequence %u / %u)", bond_seq(in_use),
	      bond_seq(other));
	r = send(hid, confirm);
	CHECK(r.status == ST_UNTESTED, "confirm after a failed record read back: status %u", r.status);

	/*
	 * 2b. A valid header with a higher sequence number appears in the
	 * blank sector behind the firmware's back (what a read that went
	 * wrong at the load would look like): the test refuses (status 3)
	 * before any erase, both sectors as they were; with it gone, the test
	 * passes again.
	 */
	{
		uint8_t fake[8];
		static const uint8_t zero[4] = {0};

		sys_put_le32(BOND_MAGIC, &fake[0]);
		sys_put_le32(seq + 5U, &fake[4]);
		CHECK(flash_write(flash, other, fake, sizeof(fake)) == 0, "2b: a stray header written");
		bond_reads = 0;
		r = send(hid, test);
		CHECK(r.status == ST_VERIFY && r.raw[18] == 3U && bond_reads == 0,
		      "2b: test with a stray header in the other sector: status %u, flags %u, %d reads", r.status,
		      r.raw[18], bond_reads);
		CHECK(bond_sector_ok(in_use, seq) && bond_seq(other) == seq + 5U,
		      "2b: the sectors changed by the refused test (sequence %u / %u)", bond_seq(in_use),
		      bond_seq(other));
		/* the stray header's magic cleared: the firmware's own test erases it with the sector below */
		CHECK(flash_write(flash, other, zero, sizeof(zero)) == 0, "2b: the stray header's magic cleared");
		CHECK(bond_seq(other) == 0U, "2b: the stray header still reads valid");
	}

	/* 3. A test that passes: the log moves to the other sector (sequence 2), the old one blank; the slots untouched. */
	bond_reads = 0;
	raises = trim_raises;
	r = send(hid, test);
	k_msleep(100);
	CHECK(r.len == REPORT_LEN && r.idx == 0xff07U && r.status == ST_OK && r.raw[17] == 1U &&
		      r.raw[18] == 11U && r.raw[21] == 'Z' && r.raw[22] == 'C',
	      "flash test: index 0x%04x, status %u, boots counted %u, flags %u", r.idx, r.status, r.raw[17],
	      r.raw[18]);
	CHECK(bond_sector_ok(other, seq + 1U) && flash_blank(in_use, STORE_SECTOR),
	      "after the test: sequence %u at the first sector (want %u), %u at the second (want 0, blank)",
	      bond_seq(other), seq + 1U, bond_seq(in_use));
	CHECK(bond_reads == reads_per_test, "the passed test made %d reads, want %d", bond_reads, reads_per_test);
	in_use = other;
	other = in_use == STORE_BASE ? STORE_BASE + STORE_SECTOR : STORE_BASE;
	seq++;
	CHECK(reboots == reboots_before, "rebooted after a flash test");
	CHECK(flash_equals(SLOT_B + 16U, &ota_img_good[16], sizeof(ota_img_good) - 16U) &&
		      flag_word(SLOT_B) == 0x544c4e4bU,
	      "slot B's image changed by the test: flag word 0x%08x", flag_word(SLOT_B));
	CHECK(flash_equals(SLOT_A + 16U, &ota_img_good2[16], sizeof(ota_img_good2) - 16U) &&
		      flag_word(SLOT_A) == 0U,
	      "slot A (the other slot) changed by the test: flag word 0x%08x", flag_word(SLOT_A));
	CHECK(!trims_raised && trim_raises == raises + 1, "a passed test: trims raised %d, %d raises (want %d)",
	      trims_raised, trim_raises, raises + 1);
	r = send(hid, version);
	CHECK(r.len == REPORT_LEN && r.raw[17] == 1U && r.raw[18] == 11U,
	      "after the test: boots counted %u, flags %u, want 1 and 11 (tested)", r.raw[17], r.raw[18]);

	/*
	 * 3a. The version reply just made says slot A (the way back) checks.
	 * One byte of its image changed behind the receiver's back (no write of
	 * the receiver's, so the slot generation stays and that reply's check
	 * would still be taken): the confirm is refused with status 14, its
	 * reply says slot A does not check, and nothing is written: the boot
	 * guard's counter sector byte for byte, the count 1, not confirmed.
	 */
	{
		static uint8_t keep[4096];
		static uint8_t bad[4096];
		static uint8_t counter[4096];

		CHECK(r.len == REPORT_LEN && (r.raw[30] & (BIT(0) | BIT(6))) == (BIT(0) | BIT(6)),
		      "after the test: flags 0x%02x, want slot A checking", r.raw[30]);
		CHECK(flash_read(flash, SLOT_A + 0x1000U, keep, sizeof(keep)) == 0 &&
			      flash_read(flash, CONFIG_TLSR_BOOT_GUARD_OFFSET, counter, sizeof(counter)) == 0,
		      "3a: read slot A and the counter sector");
		memcpy(bad, keep, sizeof(bad));
		bad[0x123] ^= 0x10U; /* inside image 2 (8,212 bytes) */
		CHECK(flash_erase(flash, SLOT_A + 0x1000U, sizeof(bad)) == 0 &&
			      flash_write(flash, SLOT_A + 0x1000U, bad, sizeof(bad)) == 0 &&
			      tlsr_slot_check(SLOT_A, NULL) != 0,
		      "3a: one byte of slot A's image changed");
		r = send(hid, confirm);
		CHECK(r.len == REPORT_LEN && r.idx == 0xff03U && r.status == ST_NO_WAY_BACK && r.raw[17] == 1U &&
			      r.raw[18] == 11U && (r.raw[30] & (BIT(0) | BIT(6))) == BIT(6),
		      "3a: confirm with slot A not checking: status %u, boots counted %u, flags %u, flags 0x%02x",
		      r.status, r.raw[17], r.raw[18], r.raw[30]);
		CHECK(flash_equals(CONFIG_TLSR_BOOT_GUARD_OFFSET, counter, sizeof(counter)),
		      "3a: the refused confirm changed the counter sector");

		/*
		 * An update's chunk 0 without "KNLT" at 8, with the gate open
		 * (slot A does not check): refused (status 15), slot A is not
		 * written, and the end command neither marks anything bootable
		 * nor reboots.
		 */
		{
			const int good_n = ARRAY_SIZE(ota_frames_good);
			uint8_t start_cmd[REPORT_LEN];
			uint8_t chunk0[REPORT_LEN];
			const uint32_t flag_a = flag_word(SLOT_A);

			memcpy(chunk0, ota_frames_good[1], REPORT_LEN);
			memcpy(&chunk0[11 + 8], "TEST", 4);
			sys_put_le16(crc16_ansi(&chunk0[9], 18), &chunk0[27]);
			command(start_cmd, 0xff01U);
			r = send(hid, start_cmd);
			CHECK(r.idx == 0U && r.status == ST_OK, "3a: start command: status %u", r.status);
			r = send(hid, chunk0);
			CHECK(r.status == ST_NOT_BOOTABLE, "3a: chunk 0 without KNLT as an update: index %u, status %u",
			      r.idx, r.status);
			r = send(hid, ota_frames_good[2]);
			CHECK(r.status == ST_INDEX, "3a: chunk 1 after the refusal: status %u", r.status);
			r = send(hid, ota_frames_good[good_n - 1]);
			CHECK(r.status == ST_END, "3a: the end command after the refusal: status %u", r.status);
			k_msleep(100);
			CHECK(reboots == reboots_before, "3a: rebooted after a refused chunk 0");
			CHECK(flash_equals(SLOT_A + 0x1000U, bad, sizeof(bad)) && flag_word(SLOT_A) == flag_a &&
				      flash_equals(SLOT_A + 16U, &ota_img_good2[16], 0x1000U - 16U),
			      "3a: slot A written after a refused chunk 0: flag word 0x%08x", flag_word(SLOT_A));
			CHECK(flash_equals(SLOT_B + 16U, &ota_img_good[16], sizeof(ota_img_good) - 16U) &&
				      flag_word(SLOT_B) == 0x544c4e4bU,
			      "3a: slot B changed after a refused chunk 0: flag word 0x%08x", flag_word(SLOT_B));
			CHECK(!trims_raised, "3a: the trims stay raised after a refused chunk 0");
		}
		CHECK(flash_erase(flash, SLOT_A + 0x1000U, sizeof(keep)) == 0 &&
			      flash_write(flash, SLOT_A + 0x1000U, keep, sizeof(keep)) == 0 &&
			      tlsr_slot_check(SLOT_A, NULL) == 0,
		      "3a: slot A put back");
	}

	/* 3b. The start command took the pass away; a second test passes and the sectors swap back (sequence 3). */
	r = send(hid, confirm);
	CHECK(r.status == ST_UNTESTED, "3b: confirm after a start command: status %u", r.status);
	bond_reads = 0;
	r = send(hid, test);
	CHECK(r.status == ST_OK && r.raw[18] == 11U, "3b: second flash test: status %u, flags %u", r.status,
	      r.raw[18]);
	CHECK(bond_sector_ok(other, seq + 1U) && flash_blank(in_use, STORE_SECTOR),
	      "3b: sequence %u at the second sector (want %u), %u at the first (want 0, blank)",
	      bond_seq(other), seq + 1U, bond_seq(in_use));
	CHECK(bond_reads == reads_per_test, "3b: the second test made %d reads, want %d", bond_reads,
	      reads_per_test);

	/* 4. Now the confirm is taken; its reply says slot A checks. */
	r = send(hid, confirm);
	CHECK(r.len == REPORT_LEN && r.idx == 0xff03U && r.status == ST_OK && r.raw[17] == 0U &&
		      r.raw[18] == 15U && r.raw[21] == 'Z' && r.raw[22] == 'C' &&
		      (r.raw[30] & (BIT(0) | BIT(6))) == (BIT(0) | BIT(6)),
	      "confirm: index 0x%04x, status %u, boots counted %u, flags %u, mark %c%c, flags 0x%02x", r.idx,
	      r.status, r.raw[17], r.raw[18], r.raw[21], r.raw[22], r.raw[30]);
	r = send(hid, version);
	CHECK(r.len == REPORT_LEN && r.raw[17] == 0U && r.raw[18] == 15U,
	      "after the confirm: boots counted %u, flags %u, want 0 and confirmed", r.raw[17],
	      r.raw[18]);

	/* 5. A second confirm is taken too and leaves the counter sector as it is. */
	{
		static uint8_t counter[4096];

		CHECK(flash_read(flash, CONFIG_TLSR_BOOT_GUARD_OFFSET, counter, sizeof(counter)) == 0,
		      "5: read the counter sector");
		r = send(hid, confirm);
		CHECK(r.len == REPORT_LEN && r.idx == 0xff03U && r.status == ST_OK && r.raw[17] == 0U &&
			      r.raw[18] == 15U,
		      "5: second confirm: status %u, boots counted %u, flags %u", r.status, r.raw[17], r.raw[18]);
		CHECK(flash_equals(CONFIG_TLSR_BOOT_GUARD_OFFSET, counter, sizeof(counter)),
		      "5: the second confirm changed the counter sector");
	}
	printk("confirm: refused before a flash test and after failed ones (a read back of the erased sector "
	       "and of a written record wrong: status 3), and with the other slot changed (status 14, nothing "
	       "written; a chunk 0 without KNLT then refused, status 15); a passed test (the bond log at "
	       "0x%05x: %d reads, the sectors swapped, both slots untouched, the trims put back) allows the "
	       "confirm, which clears the boot count and confirms the image; a second confirm leaves the "
	       "counter sector as it is\n", (unsigned int)STORE_BASE, reads_per_test);
	return failures - start;
}

/*
 * The flash test's edges (after tlsr_usb_ota_host_confirm, whose passed test
 * is still valid; slot B runs image 1, slot A holds image 2):
 * 1. a pass counts only until the next start command (0xff01 alone: the
 *    confirm is refused);
 * 2. and until chunk 0 (a passed test, a bare chunk 0, refused by the gate:
 *    the confirm is refused);
 * 3. a passed test, then one byte of the running image changed: the
 *    confirm is refused (status 12) and the counter sector stays as it is;
 * 4. an update into slot A (unlocked: it holds an image that checks) whose
 *    image changes in the flash before the end command: status 3, slot A
 *    not made bootable, slot B's flag kept, no reboot, the Zbit trims put
 *    back;
 * 5. START raises the Zbit trims; an end command with no chunk after it
 *    (status 4) puts them back;
 * 6. the OTA test's old start command (0xff04) is unknown now: status 8,
 *    nothing written, the pass of a test before it kept (the confirm is
 *    then refused for slot A's image, changed in 4, not for the test);
 * 7. the flash test in the middle of an update (unlocked, chunks 0-2 into
 *    slot A) is refused with status 9, the trims stay raised, the bond log
 *    is untouched, and the update goes on to its end (status 0, reboot);
 * 8. a record added to the sector in use behind the firmware's back (a
 *    state record naming another profile): the test refuses (status 3)
 *    with no read of the sectors and nothing erased, both headers as they
 *    were.
 */
int tlsr_usb_ota_host_test_edges(const struct tlsr_hid_info *hid)
{
	const int start = failures;
	uint8_t frame[REPORT_LEN];
	uint8_t test[REPORT_LEN];
	struct resp r;

	running = SLOT_B;
	command(test, 0xff07U);
	/* 1. */
	command(frame, 0xff01U);
	r = send(hid, frame);
	CHECK(r.status == ST_OK && r.idx == 0U, "edges: START: status %u, index 0x%04x", r.status, r.idx);
	command(frame, 0xff03U);
	r = send(hid, frame);
	CHECK(r.status == ST_UNTESTED, "edges: confirm after a pass and START: status %u", r.status);

	/* 2. */
	r = send(hid, test);
	CHECK(r.status == ST_OK, "edges: flash test: status %u", r.status);
	r = send(hid, ota_frames_good[1]);
	CHECK(r.status == ST_LOCKED, "edges: a bare chunk 0 over slot A's image: status %u", r.status);
	command(frame, 0xff03U);
	r = send(hid, frame);
	CHECK(r.status == ST_UNTESTED, "edges: confirm after a passed test and a bare chunk 0: status %u",
	      r.status);

	/* 3. */
	{
		static uint8_t counter[4096];
		static const uint8_t zero = 0U;

		r = send(hid, test);
		CHECK(r.status == ST_OK, "edges: flash test (3): status %u", r.status);
		CHECK(flash_read(flash, CONFIG_TLSR_BOOT_GUARD_OFFSET, counter, sizeof(counter)) == 0,
		      "edges: read the counter sector");
		CHECK(flash_write(flash, SLOT_B + 0x300U, &zero, 1U) == 0 && tlsr_slot_check(SLOT_B, NULL) != 0,
		      "edges: change one byte of the running image");
		command(frame, 0xff03U);
		r = send(hid, frame);
		CHECK(r.idx == 0xff03U && r.status == ST_RUNNING,
		      "edges: confirm with a running image that does not check: status %u", r.status);
		CHECK(flash_equals(CONFIG_TLSR_BOOT_GUARD_OFFSET, counter, sizeof(counter)),
		      "edges: the refused confirm changed the counter sector");
	}

	/* 4. */
	{
		const int good_n = ARRAY_SIZE(ota_frames_good);
		const int reboots_before = reboots;
		static const uint8_t zero = 0U;

		command(frame, 0xff05U);
		r = send(hid, frame);
		CHECK(r.idx == 0xff05U && r.status == ST_OK, "edges: unlock (4): status %u", r.status);
		CHECK(send_all(hid, ota_frames_good, good_n - 1, "good") == good_n - 1,
		      "edges: START and all chunks into slot A");
		CHECK(flash_write(flash, SLOT_A + 0x200U, &zero, 1U) == 0, "edges: change slot A's new image");
		r = send(hid, ota_frames_good[good_n - 1]);
		k_msleep(100);
		CHECK(r.idx == 0xff02U && r.status == ST_VERIFY,
		      "edges: end of an update whose image changed in the flash: status %u", r.status);
		CHECK(reboots == reboots_before, "edges: rebooted after a refused update");
		CHECK((flag_word(SLOT_A) & 0xffU) != 0x4bU && flag_word(SLOT_B) == 0x544c4e4bU,
		      "edges: slot flags after a refused update: A 0x%08x, B 0x%08x", flag_word(SLOT_A),
		      flag_word(SLOT_B));
		CHECK(!trims_raised, "edges: the trims stay raised after a refused update");
	}

	/* 5. */
	{
		const int raises = trim_raises;

		command(frame, 0xff01U);
		r = send(hid, frame);
		CHECK(r.status == ST_OK && trims_raised && trim_raises == raises + 1,
		      "edges: START raises the trims: status %u", r.status);
		r = send(hid, ota_frames_good[ARRAY_SIZE(ota_frames_good) - 1]);
		CHECK(r.idx == 0xff02U && r.status == ST_END, "edges: END after a bare START: status %u",
		      r.status);
		CHECK(!trims_raised, "edges: the trims stay raised after START and END with no chunk");
	}

	/* 6. */
	{
		const uint32_t seq_a = bond_seq(STORE_BASE);
		const uint32_t seq_b = bond_seq(STORE_BASE + STORE_SECTOR);

		r = send(hid, test);
		CHECK(r.status == ST_OK, "edges: flash test (6): status %u", r.status);
		command(frame, 0xff04U);
		r = send(hid, frame);
		CHECK(r.idx == 0xff04U && r.status == 8U, "edges: the old test start: index 0x%04x, status %u",
		      r.idx, r.status);
		CHECK(bond_seq(STORE_BASE) + bond_seq(STORE_BASE + STORE_SECTOR) == seq_a + seq_b + 1U,
		      "edges: the bond log changed by the old test start");
		command(frame, 0xff03U);
		r = send(hid, frame);
		/* The pass kept: the refusal is the way back's (slot A's image was changed in 4), not status 10. */
		CHECK(r.status == ST_NO_WAY_BACK, "edges: confirm after the old test start: status %u (want 14: "
		      "the pass kept, slot A's image changed in 4)", r.status);
	}

	/* 7. */
	{
		const int good_n = ARRAY_SIZE(ota_frames_good);
		const int reboots_before = reboots;
		const uint32_t seq_a = bond_seq(STORE_BASE);
		const uint32_t seq_b = bond_seq(STORE_BASE + STORE_SECTOR);

		command(frame, 0xff05U);
		r = send(hid, frame);
		CHECK(r.idx == 0xff05U && r.status == ST_OK, "edges: unlock (7): status %u", r.status);
		CHECK(send_all(hid, ota_frames_good, 4, "good") == 4, "edges: START and chunks 0-2 into slot A");
		r = send(hid, test);
		CHECK(r.idx == 0xff07U && r.status == ST_SLOT && trims_raised,
		      "edges: flash test during an update: status %u (want 9), trims raised %d", r.status,
		      trims_raised);
		CHECK(bond_seq(STORE_BASE) == seq_a && bond_seq(STORE_BASE + STORE_SECTOR) == seq_b,
		      "edges: the bond log changed by a refused test");
		CHECK(send_all(hid, &ota_frames_good[4], good_n - 4, "good") == good_n - 4,
		      "edges: the update's remaining chunks and end command");
		wait_reboot(reboots_before + 1);
		CHECK(reboots == reboots_before + 1, "edges: no reboot after the update that a refused test interrupted");
		CHECK(flash_equals(SLOT_A, ota_img_good, sizeof(ota_img_good)), "edges: slot A holds the update's image");
		trims_raised = false; /* the reboot puts them back on hardware */
	}

	/* 8. */
	{
		const uint32_t in_use = bond_seq(STORE_BASE) != 0U ? STORE_BASE : STORE_BASE + STORE_SECTOR;
		const uint32_t seq_a = bond_seq(STORE_BASE);
		const uint32_t seq_b = bond_seq(STORE_BASE + STORE_SECTOR);
		uint8_t rec[64] = {0xb1, 2};

		sys_put_le16(crc16_ccitt(0xffff, rec, 62), &rec[62]);
		CHECK(flash_write(flash, in_use + 8U + 64U, rec, sizeof(rec)) == 0, "edges: a stray state record written");
		bond_reads = 0;
		r = send(hid, test);
		CHECK(r.idx == 0xff07U && r.status == ST_VERIFY && bond_reads == 0,
		      "edges: test with a stray record in the sector in use: status %u (want 3), %d reads", r.status,
		      bond_reads);
		CHECK(bond_seq(STORE_BASE) == seq_a && bond_seq(STORE_BASE + STORE_SECTOR) == seq_b,
		      "edges: the headers changed by the refused test (%u / %u)", bond_seq(STORE_BASE),
		      bond_seq(STORE_BASE + STORE_SECTOR));
	}

	printk("flash test edges: a pass lasts until the next start or chunk 0; a running image that does not "
	       "check refuses the confirm; an update whose image changes before its end is refused; the old "
	       "test start is unknown; the test during an update is refused and the update completes; a "
	       "record the firmware did not write refuses the test\n");
	return failures - start;
}

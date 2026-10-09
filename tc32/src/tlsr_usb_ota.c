/*
 * Telink USB OTA receiver for TLSR827x keyboards.
 *
 * Takes firmware images over HID report ID 5 with the protocol the stock
 * firmware's OTA also takes (the CIDOO V75 Pro, V65 V3 and V21), so that an
 * image installed through the stock OTA can in turn be replaced, by a newer
 * build or by the stock image.
 *
 * Flash layout, shared with the stock image: two 128 KB slots at 0x00000
 * and 0x20000. The boot ROM starts the slot whose byte 8 is 0x4b (the "KNLT"
 * word at 8..11). An image goes to the slot that is not running, with byte 8
 * held at 0xff, every chunk inside that slot; the end command then checks the
 * image as the flash holds it (its size word and CRC-32, status 3 otherwise),
 * sets byte 8 of the new slot, clears bytes 8..11 of the running one and
 * reboots. An update that aborts puts back the analog fields its start
 * raised on a Zbit flash.
 *
 * Report (33 bytes, both directions):
 *   [0]      report ID 5
 *   [1]      payload length + 9
 *   [2]      0x01 (required)
 *   [3]      payload length + 7
 *   [4]      payload length + 3
 *   [5..8]   04 52 28 00
 *   [9..10]  chunk index, or a command (0xff00 version, 0xff01 start,
 *            0xff02 end, 0xff03 confirm: the host confirms the image, the
 *            boot guard's count is cleared and the image marked confirmed;
 *            the reply carries the version information with byte 17 = 0
 *            and byte 18 bit 2 set; refused with status 10 unless an OTA
 *            test passed in this boot, with status 14 unless the other
 *            slot's image checks when the confirm arrives, and with status
 *            12 unless the running image checks then (see the confirm
 *            below); a refusal writes nothing. 0xff07 flash test: the bond
 *            log's two sectors read, erased, written and read back, see
 *            below; the reply carries the version information.
 *            0xff05 unlock: the next update may go over an image that
 *            checks in the other slot, within CONFIG_TLSR_USB_OTA_ARM_MS;
 *            without it that update's chunk 0 gets status 13, see the gate
 *            below; the reply carries the version information;
 *            0xff06 battery: one measurement, nothing written, see
 *            ota_handle(); 0xff0c link counters: byte 11 the page, the
 *            reply's bytes 11..28 the 18 octets of
 *            ble_link_stats_read() from page * 18, nothing written;
 *            0xff0e raw samples of tc32_rng's sources, measurement images
 *            only: src/tlsr_rng_measure.c),
 *            little-endian. An update's chunk 0 without "KNLT" at 8 gets
 *            status 15, nothing written: only a Telink image is ever made
 *            bootable
 *   [11..26] chunk: 16 image bytes at index * 16
 *            end: the last chunk index and its complement
 *   [27..28] chunk: CRC-16/MODBUS of bytes 9..26
 * The response is the request with bytes 9..10 set to the next expected index
 * when a chunk was accepted, and to 0 for START. The version command's
 * response also carries (the stock firmware echoes zeros there):
 *   [11..14] system clock in Hz the watchdog period is based on
 *   [15..16] watchdog capture (period = capture * 2^18 / clock)
 *   [17]     boots counted at this boot without a healthy one (the boot guard's counter; 1
 *            on a first boot until the host has configured USB)
 *   [18]     bit 0: the clock was measured; bit 1: the boot guard runs; bit 2: the host
 *            has confirmed this image (the automatic healthy rule applies); bit 3: the
 *            flash test passed in this boot; bit 4: the planned-reboot mark register did
 *            not read back its cleared value at this boot, so the boot was taken as
 *            unplanned (tlsr_boot_mark_stuck())
 *   [19..20] the flash status register as the boot guard found it at boot (low byte, high byte)
 *   [23..24] the same as the guard left it (after its unlock, when it wrote one)
 *   [25]     bit 0: the guard cleared the block protection this boot; bit 1: the bits are
 *            still set after its write; bit 2: a flash write of the guard or the revert did
 *            not read back in this boot; bit 3: a flash part the unlock does not handle; bit 4:
 *            the register was read this boot (else bytes 19-20, 23-24 are 0 for that reason);
 *            bit 5: it read as busy or write-enabled (a misread), so nothing was written
 *   [26..28] the flash's JEDEC ID (manufacturer, type, capacity), 0 when not read
 *   [30..32] the other slot's check, the update gate and its unlock, the
 *            running slot, the CPU left over, the uptime (diag_info() below;
 *            the confirm's and the unlock's replies carry them too)
 *   [21..22] "ZC", marking this receiver
 * Byte 29 carries a status code (the OTA protocol's error numbers 1-6 and
 * this receiver's 7-16, 0 on success); the stock firmware leaves it as sent.
 *
 * The confirm shows four things before
 * the boot guard stops counting an image: the keys that go back to the other
 * image work (checked by hand; nothing here), the other slot's image,
 * the way back, checks (0xff03 checks its size word and CRC-32 then and
 * there, status 14 otherwise), the running image checks the same way
 * (status 12 otherwise), and the image can erase, write and read its flash
 * (the flash test passed in this boot, status 10 otherwise). telink_ota.py
 * confirm sends the flash test command, then the confirm.
 *
 * Flash test (0xff07). The bond log's two sectors (src/ble/ble_bond.c, the
 * first two of the storage partition: the BLE bonds and the profile state,
 * which the log erases and writes in use anyway), compacted with every step
 * read back: the header of the sector in use read, the other sector erased
 * and read back blank, each record written there and read back equal, its
 * header written last, then the old sector erased and read back blank. It
 * runs through the flash driver the ways back and an update use, with the
 * analog fields a Zbit flash needs for programming raised for it
 * (tlsr_spi_flash_trim_raise()) and put back after it, and it reads back
 * what it erased and wrote, so a flash that takes no erase or write, or
 * reads back something else, fails it: status 3 when a read back differs
 * (the flash driver reports no error of its own, so this is how a fault
 * shows), 7 when a flash call fails; status 9 while an update is under way
 * (its state and trims untouched). The test first reads both headers again
 * and refuses, nothing erased, unless they give the sector the load chose.
 * A pass allows 0xff03 until the next start command, chunk 0 or boot.
 * Nothing outside those two sectors is written, so no slot changes: the
 * test is the same whatever the image's size, and the image may fill its
 * slot. A power cut inside the test leaves one sector with a valid header
 * (each erase is preceded by clearing the sector's magic, the new header
 * is written last, and the old one is cleared only after it).
 *
 * Image checks: the size word at 0x18 counts the image including a trailing
 * CRC-32 (reflected, init 0xffffffff, no final XOR) of everything before it,
 * and is 16n + 4. The chunk holding that CRC is the last one.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>

#include "ble/tlsr_ble.h"
#include "tlsr_cpu_left.h"
#include "tlsr_slots.h"
#ifdef CONFIG_TLSR_CRASH_LOG
size_t tlsr_crash_log_read(size_t at, uint8_t *out, size_t n);
#endif
#ifdef CONFIG_TLSR_P24_DIAG
size_t p24_diag_read(size_t at, uint8_t *out, size_t n);
#endif
#ifdef CONFIG_BATTERY_ADC
#include "battery_adc.h"
#endif
#ifdef CONFIG_TLSR_RNG_MEASURE
#include "tlsr_rng_measure.h"
#endif
#ifdef CONFIG_TLSR_PKE_TEST
#include "tlsr_pke_test.h"
#endif
#if defined(CONFIG_TLSR_SPI_FLASH) || defined(CONFIG_TLSR_USB_OTA_SIM_TRIMS)
#include "tlsr_spi_flash.h"
#define OTA_TRIMS 1
#endif

LOG_MODULE_REGISTER(tlsr_usb_ota, CONFIG_TLSR_USB_OTA_LOG_LEVEL);

#define OTA_REPORT_ID  0x05U
#define OTA_REPORT_LEN 33U

#define OTA_IDX    9U
#define OTA_DATA   11U
#define OTA_CRC16  27U
#define OTA_STATUS 29U

#define OTA_CMD_VERSION 0xff00U
#define OTA_CMD_START   0xff01U
#define OTA_CMD_END     0xff02U
#define OTA_CMD_CONFIRM 0xff03U /* ours: the boot guard's healthy mark from the host */
/* 0xff04 was the OTA test's start, which wrote the running slot's tail: unknown now (status 8) */
#define OTA_CMD_UNLOCK  0xff05U /* ours: allow the next update over an image in the other slot */
#define OTA_CMD_BATTERY 0xff06U /* ours: one battery measurement, nothing written */
#define OTA_CMD_CRASH_LOG 0xff0dU /* ours: the boot before's crash record (CONFIG_TLSR_CRASH_LOG), nothing written */
#define OTA_CMD_P24_DIAG 0xff0bU /* ours: the 2.4G link's counters (CONFIG_TLSR_P24_DIAG), nothing written */
#define OTA_CMD_FLASH_TEST 0xff07U /* ours: the bond log's sectors read, erased, written and read back */
#define OTA_CMD_LINK_STATS 0xff0cU /* ours: the BLE and 2.4G links' counters (CONFIG_TLSR_BLE), nothing written */
#define OTA_CMD_RNG_SAMPLES 0xff0eU /* ours: raw samples of tc32_rng's sources (CONFIG_TLSR_RNG_MEASURE) */
#define OTA_CMD_PKE_TEST 0xff0fU /* ours: the P-256 engine's known answers (CONFIG_TLSR_PKE_TEST), nothing written */

#define OTA_CHUNK 16U

#define HID_SET_REPORT_OUTPUT 0x02U

BUILD_ASSERT(CONFIG_HID_INTERRUPT_EP_MPS >= OTA_REPORT_LEN,
	     "the response goes out as one interrupt packet");
BUILD_ASSERT(CONFIG_USB_HID_DEVICE_COUNT >= 2, "HID_0 is the keyboard, HID_1 the OTA interface");

/* Status codes; 1-6 are the OTA protocol's error numbers. */
enum ota_status {
	OTA_OK = 0,
	OTA_ERR_INDEX = 1,
	OTA_ERR_CRC16 = 2,
	OTA_ERR_VERIFY = 3,
	OTA_ERR_END = 4,
	OTA_ERR_IMAGE = 6,
	OTA_ERR_FLASH = 7,
	OTA_ERR_REPORT = 8,
	OTA_ERR_SLOT = 9, /* the running slot unclear at chunk 0; the flash test during an update */
	OTA_ERR_UNTESTED = 10, /* confirm before the flash test passed in this boot */
	/* 11 was: the OTA test's image does not fit in the running slot's tail */
	OTA_ERR_RUNNING = 12,  /* the running image's size word or CRC-32 does not check: no confirm */
	OTA_ERR_LOCKED = 13,   /* an update over an image that checks in the other slot, not unlocked */
	OTA_ERR_NO_WAY_BACK = 14, /* confirm while the other slot holds no image that checks */
	OTA_ERR_NOT_BOOTABLE = 15, /* an update's chunk 0 without "KNLT" at 8 */
	OTA_ERR_NO_BATTERY = 16,   /* the battery command on a build without the measurement */
};

static const uint8_t knlt[4] = {'K', 'N', 'L', 'T'};

static const struct device *flash_dev;
static const struct device *hid_dev;

K_MSGQ_DEFINE(ota_msgq, OTA_REPORT_LEN, 4, 1);

static struct {
	bool active;
	bool crc_ok;
	int32_t last;      /* last chunk written, -1 before chunk 0 */
	uint32_t end;      /* index of the chunk that holds the image CRC, 0 until chunk 1 */
	uint32_t crc;      /* crc32_ieee running value over chunks 0..end-1 */
	uint32_t target;   /* slot being written */
	uint32_t running;  /* slot the firmware runs from */
} ota;

static bool test_passed; /* the flash test passed in this boot: confirm is allowed */
static int other_ok = -1; /* the other slot's check for the version reply: -1 until made, and again once an update starts */
static uint32_t other_gen; /* tlsr_slot_generation() when other_ok was made: a write since makes it stale */

/*
 * The gate (CONFIG_TLSR_USB_OTA_GATE). The other slot holds the stock firmware
 * after the install (or the previous build): the image the boot guard, the
 * power-on chord and &prev_fw go back to, and an update overwrites it from its
 * first chunk on. So chunk 0 of an update is taken while that slot holds an
 * image that checks only after the unlock command (0xff05, which telink_ota.py
 * sends only when asked to overwrite that slot), within
 * CONFIG_TLSR_USB_OTA_ARM_MS, and uses the unlock up. No key is needed: this
 * receiver is also the way back when the keys do not work. With nothing that
 * checks there (after a cut transfer, say), the update is the way back and is
 * taken as before. The flash test the confirm runs writes the bond log's
 * sectors only and is not held up.
 */
static int64_t unlocked_until = -1; /* uptime (ms) until which an update may go over the other slot */

static bool unlocked(void)
{
	return unlocked_until >= 0 && k_uptime_get() <= unlocked_until;
}

static const uint8_t ota_report_desc[] = {
	0x05, 0x01,       /* Usage Page (Generic Desktop) */
	0x09, 0x00,       /* Usage (Undefined) */
	0xa1, 0x01,       /* Collection (Application) */
	0x85, 0x05,       /*   Report ID (5) */
	0x15, 0x00,       /*   Logical Minimum (0) */
	0x26, 0xff, 0x00, /*   Logical Maximum (255) */
	0x75, 0x08,       /*   Report Size (8) */
	0x95, 0x20,       /*   Report Count (32) */
	0x09, 0x01,       /*   Usage (0x01) */
	0x81, 0x02,       /*   Input (Data, Variable, Absolute) */
	0x09, 0x02,       /*   Usage (0x02) */
	0x91, 0x02,       /*   Output (Data, Variable, Absolute) */
	0xc0,             /* End Collection */
};

static int write_verify(uint32_t addr, const uint8_t *data, size_t len)
{
	uint8_t back[OTA_CHUNK];
	int err;

	__ASSERT_NO_MSG(len <= sizeof(back));
	tlsr_slot_touched();
	err = flash_write(flash_dev, addr, data, len);
	if (err != 0) {
		return err;
	}
	err = flash_read(flash_dev, addr, back, len);
	if (err != 0) {
		return err;
	}
	return memcmp(back, data, len) == 0 ? 0 : -EIO;
}

/* The analog fields the start raised on a Zbit flash put back (nothing when they were not raised). */
static void ota_voltage_restore(void)
{
#ifdef OTA_TRIMS
	tlsr_spi_flash_trim_restore();
#endif
}

static enum ota_status ota_abort(enum ota_status status)
{
	LOG_WRN("OTA aborted at chunk %d: status %d", ota.last, status);
	ota.active = false;
	/*
	 * An aborted update, or a start that got no chunk: the analog
	 * fields the start raised on a Zbit flash go back (nothing when
	 * they were not raised).
	 */
	ota_voltage_restore();
	return status;
}

/* Returns the status; *ack is set when the chunk is (or already was) written. */
static enum ota_status ota_chunk(uint16_t idx, uint8_t data[OTA_CHUNK], bool *ack)
{
	uint32_t addr;
	int err;

	*ack = false;
	if (idx == 0U) {
		uint32_t target;

		/* Chunk 0 starts a transfer, with or without a start command before it. */
		test_passed = false;
		ota.active = false;
		if (tlsr_slot_running_checked(&ota.running) != 0) {
			/* Never write when it is unclear which slot is running. */
			ota_voltage_restore();
			return OTA_ERR_SLOT;
		}
		/*
		 * An update installs only a Telink image: "KNLT" at 8. Before
		 * the gate: nothing is read.
		 */
		if (memcmp(&data[TLSR_SLOT_FLAG], knlt, sizeof(knlt)) != 0) {
			LOG_WRN("update refused: no KNLT at 8");
			ota_voltage_restore();
			return OTA_ERR_NOT_BOOTABLE;
		}
		target = ota.running == TLSR_SLOT_A ? TLSR_SLOT_B : TLSR_SLOT_A;
#ifdef CONFIG_TLSR_USB_OTA_GATE
		/*
		 * The gate: not over an image that checks unless unlocked
		 * (above). The generation read before the check, as the
		 * version reply does.
		 */
		const uint32_t gen = tlsr_slot_generation();

		if (tlsr_slot_check(target, NULL) == 0) {
			if (!unlocked()) {
				LOG_WRN("update refused: the other slot holds an image that checks");
				/* The version reply answers from this check. */
				other_ok = 1;
				other_gen = gen;
				ota_voltage_restore();
				return OTA_ERR_LOCKED;
			}
			unlocked_until = -1;
		}
#endif
		/*
		 * An update writes the other slot, with or without a start
		 * command before chunk 0: its image is checked again at the
		 * next version request.
		 */
		other_ok = -1;
		ota.target = target;
		ota.active = true;
		ota.crc_ok = false;
		ota.last = -1;
		ota.end = 0U;
		ota.crc = 0U;
		LOG_INF("OTA into 0x%05x", ota.target);
	} else if (!ota.active) {
		return OTA_ERR_INDEX;
	}

	if ((int32_t)idx <= ota.last) {
		/* A repeat of the last chunk: its acknowledgement was lost. */
		*ack = (int32_t)idx == ota.last;
		return *ack ? OTA_OK : OTA_ERR_INDEX;
	}
	if ((int32_t)idx != ota.last + 1) {
		return ota_abort(OTA_ERR_INDEX);
	}

	if (idx == 1U) {
		uint32_t size = sys_get_le32(&data[TLSR_SLOT_SIZE_WORD - OTA_CHUNK]);

		if ((size & 0xfU) != 4U || size < 0x24U || size > TLSR_SLOT_SIZE) {
			LOG_WRN("image size %u", size);
			return ota_abort(OTA_ERR_IMAGE);
		}
		ota.end = size / OTA_CHUNK;
	}

	if (ota.end != 0U && idx == ota.end) {
		if (sys_get_le32(data) != ~ota.crc) {
			LOG_WRN("image CRC 0x%08x, computed 0x%08x", sys_get_le32(data), ~ota.crc);
			return ota_abort(OTA_ERR_IMAGE);
		}
		ota.crc_ok = true;
	} else if (ota.end != 0U && idx > ota.end) {
		return ota_abort(OTA_ERR_IMAGE);
	} else if (idx == 0U) {
		/* Bytes 8..11 count as "KNLT", as tlsr_slot_check() counts them. */
		uint8_t first[OTA_CHUNK];

		memcpy(first, data, OTA_CHUNK);
		memcpy(&first[TLSR_SLOT_FLAG], knlt, sizeof(knlt));
		ota.crc = crc32_ieee_update(ota.crc, first, OTA_CHUNK);
	} else {
		ota.crc = crc32_ieee_update(ota.crc, data, OTA_CHUNK);
	}

	addr = ota.target + idx * OTA_CHUNK;
	/*
	 * Never outside what is being written, whatever the state above says:
	 * only the other slot, never the running one.
	 */
	if (ota.target == ota.running || addr + OTA_CHUNK > ota.target + TLSR_SLOT_SIZE) {
		LOG_ERR("OTA chunk at 0x%05x outside the target", addr);
		return ota_abort(OTA_ERR_INDEX);
	}
	if (addr % TLSR_SECTOR == 0U) {
		tlsr_slot_touched();
		err = flash_erase(flash_dev, addr, TLSR_SECTOR);
		if (err != 0) {
			return ota_abort(OTA_ERR_FLASH);
		}
	}
	if (idx == 0U) {
		/* Not bootable until the end command. */
		data[TLSR_SLOT_FLAG] = 0xffU;
	}
	err = write_verify(addr, data, OTA_CHUNK);
	if (err != 0) {
		return ota_abort(err == -EIO ? OTA_ERR_VERIFY : OTA_ERR_FLASH);
	}

	ota.last = idx;
	*ack = true;
	return OTA_OK;
}

/* The flash test (the header's rules); a pass allows the confirm. */
#if IS_ENABLED(CONFIG_TLSR_BOND_LOG)
static enum ota_status ota_flash_test(void)
{
	int err;

	test_passed = false;
	if (ota.active) {
		/* not in the middle of an update: its trims and its state stay as they are */
		return OTA_ERR_SLOT;
	}
#ifdef OTA_TRIMS
	tlsr_spi_flash_trim_restore();
	tlsr_spi_flash_trim_raise();
#endif
	err = ble_bond_flash_test();
	ota_voltage_restore();
	test_passed = err == 0;
	LOG_INF("flash test: %s (%d)", test_passed ? "passed" : "failed", err);
	return err == 0 ? OTA_OK : (err == -EILSEQ ? OTA_ERR_VERIFY : OTA_ERR_FLASH);
}
#endif

static enum ota_status ota_end(uint16_t idx, uint16_t inv)
{
	static const uint8_t valid = TLSR_SLOT_FLAG_OK;
	static const uint8_t cleared[4] = {0};

	if (!ota.active || (uint16_t)(idx ^ inv) != 0xffffU || (int32_t)idx != ota.last ||
	    !ota.crc_ok) {
		return ota_abort(OTA_ERR_END);
	}
	/*
	 * The image as the flash holds it must check (its size word and CRC-32,
	 * bytes 8..11 taken as "KNLT") before it is made bootable: the
	 * read-back of each chunk does not cover a read that went wrong there.
	 */
	if (tlsr_slot_check(ota.target, NULL) != 0) {
		LOG_WRN("update: the image in slot 0x%05x does not check", ota.target);
		return ota_abort(OTA_ERR_VERIFY);
	}
	if (write_verify(ota.target + TLSR_SLOT_FLAG, &valid, 1) != 0) {
		return ota_abort(OTA_ERR_FLASH);
	}
	if (write_verify(ota.running + TLSR_SLOT_FLAG, cleared, sizeof(cleared)) != 0) {
		LOG_ERR("old slot 0x%05x still marked bootable", ota.running);
	}
	ota.active = false;
	LOG_INF("OTA done: %d chunks into slot 0x%05x", ota.last + 1, ota.target);
	return OTA_OK;
}

/*
 * Diagnostics in the version reply's bytes
 * 30..32, which the request leaves 0 (byte 32 = 1 asks for the CPU figure):
 *   [30] bit 0: the image in the other slot checks (its size word and CRC-32,
 *        bytes 8..11 taken as KNLT; checked at the first version request
 *        after boot or after the slot generation changed, and again at each
 *        confirm that follows a passed test), which every way back needs: the boot
 *        guard, the chord, &prev_fw; bit 1: an update over the
 *        other slot's image is unlocked now; bit 2: the update gate is built
 *        in; bits 3-5, the planned-reboot mark the boot guard found at this
 *        boot (tlsr_slots.c, analog 0x3c): bit 3 this image's (a reboot the
 *        firmware asked for, into it), bit 4 another image's (a mark that
 *        stayed through the resets since it was left: a power-on clears it),
 *        bit 5 bits 3 and 4 are reported (the boot guard is built in); bit 6:
 *        bytes 30..32 carry these; bit 7: this image runs from slot B
 *        (register 0x63e, tlsr_slots.c)
 *   [31] the share of the CPU the lowest-priority thread got over 100 ms, in
 *        percent, when the request asked for it, else 0xff
 *   [32] uptime in seconds, 255 from 255 s on: a smaller value after the
 *        cable was out means the chip was powered off in between
 */
#define DIAG_FLAGS    30U
#define DIAG_CPU      31U
#define DIAG_UPTIME   32U
#define DIAG_OTHER_OK BIT(0)
#define DIAG_UNLOCKED BIT(1) /* an update over the other slot's image is unlocked now */
#define DIAG_GATE     BIT(2) /* the gate is built in (CONFIG_TLSR_USB_OTA_GATE) */
#define DIAG_PLANNED  BIT(3) /* this boot: a reboot the firmware asked for, into this image */
#define DIAG_MARK_OTHER BIT(4) /* at this boot the mark register held another image's mark */
#define DIAG_MARK     BIT(5) /* bits 3 and 4 are reported */
#define DIAG_PRESENT  BIT(6)
#define DIAG_SLOT_B   BIT(7)
#define DIAG_ASK_CPU  0x01U
static void diag_info(uint8_t r[OTA_REPORT_LEN])
{
	const bool ask_cpu = r[DIAG_UPTIME] == DIAG_ASK_CPU;
	int64_t up = k_uptime_get() / 1000;

	if (other_ok < 0 || other_gen != tlsr_slot_generation()) {
		/* The generation first: a write during the check makes the next reply check again. */
		other_gen = tlsr_slot_generation();
		other_ok = tlsr_slot_check(tlsr_slot_other(), NULL) == 0 ? 1 : 0;
	}
	r[DIAG_FLAGS] = DIAG_PRESENT | (other_ok == 1 ? DIAG_OTHER_OK : 0U) |
			(tlsr_slot_running() == TLSR_SLOT_B ? DIAG_SLOT_B : 0U) |
			(IS_ENABLED(CONFIG_TLSR_USB_OTA_GATE) ? DIAG_GATE : 0U) |
			(unlocked() ? DIAG_UNLOCKED : 0U);
#ifdef CONFIG_TLSR_BOOT_GUARD
	r[DIAG_FLAGS] |= DIAG_MARK |
			 (tlsr_boot_mark() == TLSR_BOOT_MARK_PLANNED ? DIAG_PLANNED : 0U) |
			 (tlsr_boot_mark() == TLSR_BOOT_MARK_OTHER ? DIAG_MARK_OTHER : 0U);
#endif
	r[DIAG_CPU] = ask_cpu ? tlsr_cpu_left_percent() : 0xffU;
	r[DIAG_UPTIME] = (uint8_t)MIN(up, 255);
}

static void version_info(uint8_t r[OTA_REPORT_LEN])
{
	diag_info(r);
#ifdef CONFIG_TLSR_BOOT_GUARD
	uint32_t sclk_hz;
	uint16_t capture;
	uint8_t resets;
	bool measured;
	bool confirmed;

	tlsr_boot_guard_info(&sclk_hz, &capture, &resets, &measured, &confirmed);
	sys_put_le32(sclk_hz, &r[11]);
	sys_put_le16(capture, &r[15]);
	r[17] = resets;
	r[18] = (measured ? BIT(0) : 0U) | BIT(1) | (confirmed ? BIT(2) : 0U) |
		(test_passed ? BIT(3) : 0U) | (tlsr_boot_mark_stuck() ? BIT(4) : 0U);
	{
		uint16_t status_boot;
		uint16_t status_now;
		uint8_t flags;
		uint32_t mid;

		tlsr_boot_guard_flash_info(&status_boot, &status_now, &flags, &mid);
		sys_put_le16(status_boot, &r[19]);
		sys_put_le16(status_now, &r[23]);
		r[25] = flags;
		r[26] = (uint8_t)mid;
		r[27] = (uint8_t)(mid >> 8);
		r[28] = (uint8_t)(mid >> 16);
	}
#endif
	r[21] = 'Z';
	r[22] = 'C';
}

/* Turns the request into the response; returns true when the device should reboot. */
static bool ota_handle(uint8_t r[OTA_REPORT_LEN])
{
	enum ota_status status = OTA_OK;
	bool reboot = false;
	uint16_t idx = sys_get_le16(&r[OTA_IDX]);

	if (r[0] != OTA_REPORT_ID || r[2] != 0x01U) {
		status = OTA_ERR_REPORT;
	} else if (idx == OTA_CMD_VERSION) {
		version_info(r);
	} else if (idx == OTA_CMD_START) {
		/* START is answered with index 0, as the protocol has it. */
		/* A pass counts only until the next transfer starts. */
		test_passed = false;
#ifdef OTA_TRIMS
		/*
		 * Before programming, on a Zbit flash: analog 0x09 and 0x0c raised
		 * until the reboot after an update (or the next start, if this one
		 * gets no chunk).
		 */
		tlsr_spi_flash_trim_restore();
		tlsr_spi_flash_trim_raise();
#endif
		ota.active = false;
		sys_put_le16(0U, &r[OTA_IDX]);
	} else if (idx == OTA_CMD_END) {
		status = ota_end(sys_get_le16(&r[OTA_DATA]), sys_get_le16(&r[OTA_DATA + 2]));
		reboot = status == OTA_OK;
#if IS_ENABLED(CONFIG_TLSR_BOND_LOG)
	} else if (idx == OTA_CMD_FLASH_TEST) {
		/* The flash test; the reply carries the version information (byte 18 bit 3 on a pass). */
		status = ota_flash_test();
		version_info(r);
#endif
#ifdef CONFIG_TLSR_USB_OTA_GATE
	} else if (idx == OTA_CMD_UNLOCK) {
		/* The next chunk 0 of an update may go over the other slot's image (the gate, above). */
		unlocked_until = k_uptime_get() + CONFIG_TLSR_USB_OTA_ARM_MS;
		LOG_WRN("an update over the other slot's image is unlocked for %d ms",
			CONFIG_TLSR_USB_OTA_ARM_MS);
		version_info(r);
#endif
	} else if (idx == OTA_CMD_CONFIRM) {
#ifdef CONFIG_TLSR_BOOT_GUARD
		/*
		 * Only once the flash test passed in this boot (the image can
		 * erase, write and read flash) and while the other slot's image checks, read now, not
		 * the version reply's earlier check (the way back is there): an
		 * image that fails either is never confirmed, so the boot guard can
		 * still take the keyboard back. The count is cleared and the image
		 * marked confirmed now, in this thread.
		 */
		if (!test_passed) {
			status = OTA_ERR_UNTESTED;
		} else {
			/* The version reply answers from this check too, as in diag_info(). */
			other_gen = tlsr_slot_generation();
			other_ok = tlsr_slot_check(tlsr_slot_other(), NULL) == 0 ? 1 : 0;
			if (other_ok != 1) {
				LOG_WRN("confirm refused: the other slot holds no image that checks");
				status = OTA_ERR_NO_WAY_BACK;
			} else if (tlsr_slot_check(tlsr_slot_running(), NULL) != 0) {
				/* Not an image anything has damaged. */
				LOG_WRN("confirm refused: the running image does not check");
				status = OTA_ERR_RUNNING;
			} else if (tlsr_boot_guard_confirm() != 0) {
				status = OTA_ERR_FLASH;
			}
		}
		version_info(r);
#else
		status = OTA_ERR_REPORT;
#endif
	} else if (idx == OTA_CMD_BATTERY) {
		/*
		 * One measurement as the BLE stack makes it (src/battery_adc.c):
		 * the millivolts, (raw * 590 >> 9) + 71 (11-12), the averaged ADC
		 * code (13-14), power in and charging (15 bits 0 and 1), the
		 * percentage (16), "ZC" (21-22). Nothing is written.
		 */
#ifdef CONFIG_BATTERY_ADC
		struct cidoo_battery_reading b;

		if (cidoo_battery_read(&b) != 0) {
			status = OTA_ERR_NO_BATTERY;
		} else {
			sys_put_le16(b.mv, &r[11]);
			sys_put_le16(b.raw, &r[13]);
			r[15] = (b.power ? BIT(0) : 0U) | (b.charging ? BIT(1) : 0U);
			r[16] = b.percent;
			r[21] = 'Z';
			r[22] = 'C';
		}
#else
		status = OTA_ERR_NO_BATTERY;
#endif
#ifdef CONFIG_TLSR_CRASH_LOG
	} else if (idx == OTA_CMD_CRASH_LOG) {
		/* Byte 11: the page; bytes 11..28: 18 octets of the record from page * 18.
		 * Byte 11 = 0xee with byte 12 = 0x5a instead: a fatal error now (k_oops), to try the record;
		 * with 0x5b: this thread busy for good with interrupts on (a hang above the feeder's priority). */
		if (r[OTA_DATA] == 0xeeU && r[OTA_DATA + 1] == 0x5aU) {
			k_oops();
		}
		if (r[OTA_DATA] == 0xeeU && r[OTA_DATA + 1] == 0x5bU) {
			for (;;) {
				k_busy_wait(1000);
			}
		}
		uint8_t page = r[OTA_DATA];

		memset(&r[OTA_DATA], 0, 18);
		(void)tlsr_crash_log_read((size_t)page * 18U, &r[OTA_DATA], 18);
#endif
#ifdef CONFIG_TLSR_P24_DIAG
	} else if (idx == OTA_CMD_P24_DIAG) {
		/* Byte 11 of the request: the page; the reply's bytes 11..28: 18
		 * octets of the counters from page * 18 (zeros past their end). */
		uint8_t page = r[OTA_DATA];

		memset(&r[OTA_DATA], 0, 18);
		(void)p24_diag_read((size_t)page * 18U, &r[OTA_DATA], 18);
#endif
#ifdef CONFIG_TLSR_RNG_MEASURE
	} else if (idx == OTA_CMD_RNG_SAMPLES) {
		/* Raw samples of tc32_rng's sources (src/tlsr_rng_measure.c); nothing written. */
		if (tlsr_rng_measure(r) != 0) {
			status = OTA_ERR_REPORT;
		}
#endif
#ifdef CONFIG_TLSR_PKE_TEST
	} else if (idx == OTA_CMD_PKE_TEST) {
		/* Byte 11 of the request: the page; the reply's bytes 11..28: 18 octets of
		 * the P-256 engine's record (src/tlsr_pke_test.c) from page * 18 (zeros past
		 * its end). Page 0 runs the operations; nothing written. */
		uint8_t page = r[OTA_DATA];

		memset(&r[OTA_DATA], 0, 18);
		(void)tlsr_pke_test_read((size_t)page * 18U, &r[OTA_DATA], 18);
#endif
#if IS_ENABLED(CONFIG_TLSR_BLE)
	} else if (idx == OTA_CMD_LINK_STATS) {
		/* Byte 11 of the request: the page; the reply's bytes 11..28: 18 octets of
		 * ble_link_stats_read() from page * 18 (zeros past its end). */
		uint8_t page = r[OTA_DATA];

		memset(&r[OTA_DATA], 0, 18);
		(void)ble_link_stats_read((size_t)page * 18U, &r[OTA_DATA], 18);
#endif
	} else if (idx >= OTA_CMD_VERSION) {
		status = OTA_ERR_REPORT;
	} else if (crc16_ansi(&r[OTA_IDX], 2U + OTA_CHUNK) != sys_get_le16(&r[OTA_CRC16])) {
		/* Corrupted in transit: no acknowledgement, the host sends it again. */
		status = OTA_ERR_CRC16;
	} else {
		bool ack;

		status = ota_chunk(idx, &r[OTA_DATA], &ack);
		if (ack) {
			sys_put_le16(idx + 1U, &r[OTA_IDX]);
		}
	}

	r[OTA_STATUS] = status;
	return reboot;
}

static void ota_respond(const uint8_t r[OTA_REPORT_LEN])
{
	for (int i = 0; i < 100; i++) {
		uint32_t written;

		if (hid_int_ep_write(hid_dev, r, OTA_REPORT_LEN, &written) == 0) {
			return;
		}
		k_msleep(1);
	}
	LOG_WRN("response dropped");
}

/* A request of the firmware's own in the thread's queue (never a report, whose byte 0 is OTA_REPORT_ID). */
#define OTA_OWN_CPU_LEFT 0x00U

static void (*cpu_left_cb)(uint8_t percent);

void tlsr_usb_ota_cpu_left_async(void (*done)(uint8_t percent))
{
	uint8_t r[OTA_REPORT_LEN] = {OTA_OWN_CPU_LEFT};

	cpu_left_cb = done;
	(void)k_msgq_put(&ota_msgq, r, K_NO_WAIT);
}

static void ota_thread(void *p1, void *p2, void *p3)
{
	uint8_t r[OTA_REPORT_LEN];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		(void)k_msgq_get(&ota_msgq, r, K_FOREVER);
		if (r[0] == OTA_OWN_CPU_LEFT) {
			/* the status display's measurement, in this thread (status_display.c); not in
			 * the middle of an update, whose chunks it would hold up for 100 ms */
			if (cpu_left_cb != NULL && !ota.active) {
				cpu_left_cb(tlsr_cpu_left_percent());
			}
			continue;
		}

		bool reboot = ota_handle(r);

		ota_respond(r);
		if (reboot) {
			/*
			 * Let the host take the response. The new image's tag is its
			 * CRC-32 word, which its last chunk carried and was checked
			 * against the running CRC (ota.crc stays as it was then).
			 */
			k_msleep(50);
			tlsr_reboot(~ota.crc);
		}
	}
}

K_THREAD_DEFINE(tlsr_usb_ota, CONFIG_TLSR_USB_OTA_STACK_SIZE, ota_thread, NULL, NULL, NULL,
		CONFIG_TLSR_USB_OTA_THREAD_PRIORITY, 0, 0);

/* Runs in the USB stack's control transfer context: queue and return. */
static int ota_set_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
			  uint8_t **data)
{
	uint8_t r[OTA_REPORT_LEN] = {0};

	ARG_UNUSED(dev);

	if ((setup->wValue >> 8) != HID_SET_REPORT_OUTPUT ||
	    (setup->wValue & 0xffU) != OTA_REPORT_ID || *len < (int32_t)OTA_STATUS) {
		return -ENOTSUP;
	}
	memcpy(r, *data, MIN((size_t)*len, sizeof(r)));
	return k_msgq_put(&ota_msgq, r, K_NO_WAIT);
}

static const struct hid_ops ota_ops = {
	.set_report = ota_set_report,
};

static int ota_init(void)
{
	flash_dev = tlsr_slot_flash();
	hid_dev = device_get_binding(CONFIG_USB_HID_DEVICE_NAME "_1");
	if (hid_dev == NULL || !device_is_ready(flash_dev)) {
		LOG_ERR("no HID_1 or flash device");
		return -ENODEV;
	}
	usb_hid_register_device(hid_dev, ota_report_desc, sizeof(ota_report_desc), &ota_ops);
	return usb_hid_init(hid_dev);
}

SYS_INIT(ota_init, APPLICATION, CONFIG_TLSR_USB_OTA_INIT_PRIORITY);

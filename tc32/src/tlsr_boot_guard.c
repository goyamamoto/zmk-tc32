/*
 * Boot guard: go back to the image in the other slot when this one keeps
 * hanging.
 *
 * - The Timer2 watchdog runs from POST_KERNEL on. The lowest-priority thread
 *   feeds it, so a hang, a fatal error (the kernel halts) or a thread that
 *   never yields resets the chip.
 * - Until the host has confirmed the image, each boot the firmware did not
 *   ask for adds one to a counter in flash; a confirmed image counts no boot
 *   and writes nothing at boot (its way back is the power-on chord or
 *   &prev_fw, and no revert starts on its own after a reset nobody asked
 *   for)
 *   (TLSR_BOOT_GUARD_COUNT_ALL_BOOTS: watchdog resets, power-ons and other
 *   resets alike; without it, only boots after a watchdog reset, reg 0x72
 *   bit 0): one 0x00 byte per boot in the counter sector, after a tag word
 *   naming the image the boots belong to (its trailing CRC-32). A counter
 *   tagged for another image is dropped. A reboot this firmware asks for
 *   (tlsr_reboot()) leaves a mark for the image it boots in analog register
 *   0x3c, which watchdog and software resets keep (tlsr_slots.c), and that
 *   boot is not counted, even if a software reset also sets the watchdog
 *   flag, and does not read the chord.
 * - The watchdog period comes from the system clock, measured at boot by
 *   counting Timer2 against the system timer.
 * - A healthy boot clears the counter. Until the host has confirmed the
 *   image (the OTA report's 0xff03, tlsr_usb_ota.c, which takes it only
 *   after the flash test passed in the same boot and while the other slot's
 *   image checks when it arrives), only that confirm does
 *   it, and it also writes a confirmed byte under the tag, which stays.
 *   From then on no boot is counted; the automatic rule (the
 *   TLSR_BOOT_GUARD_HEALTHY choice: the host configuring the USB device,
 *   tlsr_boot_guard_usb.c, or TLSR_BOOT_GUARD_HEALTHY_MS of running) only
 *   clears a count a flash fault could have left. A counter tagged for
 *   another image is dropped, so a new image starts unconfirmed.
 * - When it reaches TLSR_BOOT_GUARD_RESETS, the image in the other slot (the
 *   stock firmware after the first install, or the previous build) is made
 *   the one that boots, if it passes its CRC check.
 *
 * So an unconfirmed image that hangs is left after that many watchdog
 * resets, and one that runs but does not work after that many power-ons in
 * a row, with no tool. A confirmed image that stops working is left by the
 * power-on chord (read before the count, or right after it with
 * TLSR_BOOT_GUARD_CHORD_AFTER_COUNT) or &prev_fw.
 * - With TLSR_BOOT_GUARD_EARLY the counting and the revert run from the
 *   board's early-init hook (tlsr_boot_guard_early()), right after the SoC
 *   setup and before any device or thread, through the flash routines of
 *   tlsr_spi_flash.c: an image that hangs anywhere after the SoC setup
 *   is reset by the SoC's early watchdog, counted here, and left after
 *   TLSR_BOOT_GUARD_RESETS of them. The POST_KERNEL step then only starts
 *   the watchdog and the healthy rule. Only a hang inside the SoC setup
 *   itself (whose waits are bounded and reboot) escapes the count.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/drivers/flash.h>
#if defined(CONFIG_TLSR_BOOT_GUARD_CHORD) || defined(CONFIG_TLSR_BOOT_GUARD_UNLOCK)
#include "tlsr_spi_flash.h" /* tlsr_spi_flash_delay_us, tlsr_spi_flash_unlock */
#endif
#include <errno.h>
#include <limits.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "tlsr_slots.h"

LOG_MODULE_REGISTER(tlsr_boot_guard, CONFIG_TLSR_SLOTS_LOG_LEVEL);

#define COUNTER     CONFIG_TLSR_BOOT_GUARD_OFFSET
#define COUNTER_TAG COUNTER
#define COUNTER_CONFIRMED (COUNTER + 4U) /* 0xff until the host confirms the image, then 0x00 */
#define COUNTER_CNT (COUNTER + 16U)
#define COUNTER_MAX 64U

BUILD_ASSERT(CONFIG_TLSR_BOOT_GUARD_OFFSET % TLSR_SECTOR == 0U);
BUILD_ASSERT(CONFIG_TLSR_BOOT_GUARD_RESETS < COUNTER_MAX);

#ifndef CONFIG_TLSR_SLOTS_SIM
/* Timer2 as watchdog (DS-TLSR8278 5.1.6): reset when tick[31:18] reaches the capture. */
#define TIMER_CTRL             0x00800620U /* 32 bits: 0x620-0x623 */
#define TIMER_STATUS           0x00800623U
#define TIMER2_TICK            0x00800638U
#define RESET_STATUS           0x00800072U
#define TIMER2_EN              BIT(6)
#define TIMER2_MODE            (BIT(7) | BIT(8))
#define WATCHDOG_CAPTURE_SHIFT 9
#define WATCHDOG_CAPTURE       GENMASK(22, 9)
#define WATCHDOG_EN            BIT(23)
#define TIMER_STATUS_ALL       GENMASK(31, 24)
#define TIMER_STATUS_WATCHDOG  BIT(3)
#define RESET_STATUS_WATCHDOG  BIT(0)

/* The system timer: 16 MHz from the 24 MHz crystal (2/3), the kernel's clock. */
#define STIMER_COUNT 0x00800740U
#endif

/*
 * Timer2 in mode 0 counts the system clock (DS-TLSR8278 5.1.2), so the
 * watchdog period depends on it. The SoC code sets the clock select register
 * 0x66 to 0x20, which DS-TLSR8278 4.2 reads as 48 MHz. Rather
 * than rely on that, the guard counts Timer2 against the system timer for
 * MEASURE_TICKS and takes the capture from the result. A result outside
 * SCLK_MIN..SCLK_MAX falls back to CONFIG_TLSR_BOOT_GUARD_SCLK_HZ.
 */
#define STIMER_HZ     16000000U
#define MEASURE_TICKS (STIMER_HZ / 500U) /* 2 ms */
#define SCLK_MIN      4000000U
#define SCLK_MAX      64000000U
#define MEASURE_LIMIT (4U * (SCLK_MAX / 500U)) /* Timer2 ticks: 8 ms at SCLK_MAX */
#define WATCHDOG_CAPTURE_MAX (BIT(14) - 1U)

BUILD_ASSERT((((uint64_t)CONFIG_TLSR_BOOT_GUARD_WATCHDOG_MS *
	       (CONFIG_TLSR_BOOT_GUARD_SCLK_HZ / 1000U)) >> 18) > 0U,
	     "watchdog period too short for the fallback clock");

static struct {
	uint32_t sclk_hz;
	uint16_t capture;
	uint8_t resets;
	bool measured;
	bool confirmed;
	uint16_t status_boot; /* the flash status register as found and as left (0: not read) */
	uint16_t status_now;
	int unlock;           /* tlsr_spi_flash_unlock()'s result, INT_MIN when it did not run */
} guard_info = {.unlock = INT_MIN};

/*
 * The flash's block protection: the stock image in the other slot locks the
 * low 256 KB (both slots) with status 0x18 at boot and unlocks at its OTA
 * start; a ZMK image that finds the slots locked could neither count nor go
 * back. So the first thing the guard does, before any flash write, is the
 * unlock for the part (tlsr_spi_flash_unlock(): read the status, clear
 * the block-protect bits, read again). Nothing is
 * written when no bit is set. The register as found and as left goes into
 * the OTA version reply, so the first `info` on hardware shows it.
 */
static void flash_unlock(void)
{
#if defined(CONFIG_TLSR_BOOT_GUARD_UNLOCK)
	guard_info.unlock = tlsr_spi_flash_unlock(&guard_info.status_boot, &guard_info.status_now);
	switch (guard_info.unlock) {
	case 1:
		LOG_WRN("flash status 0x%04x: block protection cleared (now 0x%04x)", guard_info.status_boot,
			guard_info.status_now);
		break;
	case 0:
		LOG_INF("flash status 0x%04x: not locked", guard_info.status_boot);
		break;
	case -ENOTSUP:
		LOG_WRN("flash status 0x%04x: a part the SDK's tables do not cover, not unlocked",
			guard_info.status_boot);
		break;
	case -EACCES:
		LOG_ERR("flash status 0x%04x reads as busy or write-enabled (a misread?): nothing written",
			guard_info.status_now);
		break;
	default:
		LOG_ERR("flash status 0x%04x: still 0x%04x after the unlock", guard_info.status_boot,
			guard_info.status_now);
		break;
	}
#endif
}

#ifdef CONFIG_TLSR_SLOTS_SIM
static uint32_t sim_mid;

void tlsr_boot_guard_sim_flash(uint16_t status_boot, uint16_t status_now, int unlock, uint32_t mid)
{
	guard_info.status_boot = status_boot;
	guard_info.status_now = status_now;
	guard_info.unlock = unlock;
	sim_mid = mid;
}
#endif

void tlsr_boot_guard_flash_info(uint16_t *status_boot, uint16_t *status_now, uint8_t *flags,
				uint32_t *mid)
{
	*status_boot = guard_info.status_boot;
	*status_now = guard_info.status_now;
	*flags = (guard_info.unlock == 1 ? BIT(0) : 0U) | (guard_info.unlock == -EIO ? BIT(1) : 0U) |
		 (tlsr_slot_write_failed() ? BIT(2) : 0U) | (guard_info.unlock == -ENOTSUP ? BIT(3) : 0U) |
		 (guard_info.unlock != INT_MIN ? BIT(4) : 0U) | (guard_info.unlock == -EACCES ? BIT(5) : 0U);
#if defined(CONFIG_TLSR_BOOT_GUARD_UNLOCK)
	*mid = guard_info.unlock != INT_MIN ? tlsr_spi_flash_jedec_id() : 0U;
#elif defined(CONFIG_TLSR_SLOTS_SIM)
	*mid = guard_info.unlock != INT_MIN ? sim_mid : 0U;
#else
	*mid = 0U;
#endif
}

static bool reset_was_watchdog(void)
{
#ifdef CONFIG_TLSR_SLOTS_SIM
	return tlsr_slots_sim_reset_was_watchdog();
#else
	bool wd = (sys_read8(RESET_STATUS) & RESET_STATUS_WATCHDOG) != 0U;

	sys_write8(RESET_STATUS_WATCHDOG, RESET_STATUS);
	return wd;
#endif
}

/*
 * The bit-serial 64-bit division of the TC32 runtime (compiler_builtins.c),
 * with the registers of __aeabi_uldivmod: dividend in r0:r1, divisor in r2:r3,
 * quotient in r0:r1. measure_sclk() divides with it rather than with the
 * __aeabi_uldivmod the rest of the firmware calls, which with
 * CONFIG_TC32_FAST_DIV64 is another division (without it, the same function).
 */
uint64_t __tc32_uldivmod_proven(uint64_t dividend, uint64_t divisor);

/* The system clock in Hz: Timer2 ticks over MEASURE_TICKS of the system timer. */
static uint32_t measure_sclk(void)
{
#ifdef CONFIG_TLSR_SLOTS_SIM
	return tlsr_slots_sim_sclk_hz();
#else
	unsigned int key = irq_lock();
	/* Timer2 in system clock mode, watchdog off; leave the status byte alone. */
	uint32_t ctrl = sys_read32(TIMER_CTRL) &
			~(TIMER2_MODE | WATCHDOG_CAPTURE | WATCHDOG_EN | TIMER_STATUS_ALL);
	uint32_t t0, t1, c0, c1;

	sys_write32(ctrl | TIMER2_EN, TIMER_CTRL);
	t0 = sys_read32(STIMER_COUNT);
	c0 = sys_read32(TIMER2_TICK);
	/*
	 * Bounded: this runs before the watchdog starts, so a system timer that
	 * does not count must not hang the boot. Timer2 counts the system clock,
	 * so MEASURE_LIMIT Timer2 ticks is far more than MEASURE_TICKS at any
	 * clock up to SCLK_MAX. Giving up returns 0: the configured clock is used.
	 */
	do {
		c1 = sys_read32(TIMER2_TICK);
		t1 = sys_read32(STIMER_COUNT);
	} while (t1 - t0 < MEASURE_TICKS && c1 - c0 < MEASURE_LIMIT);
	irq_unlock(key);
	if (t1 - t0 < MEASURE_TICKS) {
		return 0U;
	}
	return (uint32_t)__tc32_uldivmod_proven((uint64_t)(c1 - c0) * STIMER_HZ, t1 - t0);
#endif
}

static void watchdog_setup(void)
{
	uint32_t hz = measure_sclk();
	uint64_t capture;

	guard_info.measured = hz >= SCLK_MIN && hz <= SCLK_MAX;
	guard_info.sclk_hz = guard_info.measured ? hz : CONFIG_TLSR_BOOT_GUARD_SCLK_HZ;
	capture = ((uint64_t)CONFIG_TLSR_BOOT_GUARD_WATCHDOG_MS * (guard_info.sclk_hz / 1000U)) >> 18;
	guard_info.capture = (uint16_t)CLAMP(capture, 1U, WATCHDOG_CAPTURE_MAX);
	if (guard_info.measured) {
		LOG_INF("system clock %u Hz, watchdog capture %u", guard_info.sclk_hz,
			guard_info.capture);
	} else {
		LOG_WRN("system clock measured as %u Hz; using %u Hz", hz, guard_info.sclk_hz);
	}
}

static void watchdog_start(void)
{
#ifdef CONFIG_TLSR_SLOTS_SIM
	tlsr_slots_sim_watchdog(true);
#else
	/* Timer2 in system clock mode; leave the write-one-to-clear status byte alone. */
	uint32_t ctrl = sys_read32(TIMER_CTRL) & ~(TIMER2_MODE | WATCHDOG_CAPTURE | TIMER_STATUS_ALL);

	sys_write32(0U, TIMER2_TICK);
	sys_write32(ctrl | TIMER2_EN | ((uint32_t)guard_info.capture << WATCHDOG_CAPTURE_SHIFT) |
			    WATCHDOG_EN,
		    TIMER_CTRL);
#endif
}

void tlsr_boot_guard_info(uint32_t *sclk_hz, uint16_t *capture, uint8_t *resets, bool *measured,
			  bool *confirmed)
{
	*sclk_hz = guard_info.sclk_hz;
	*capture = guard_info.capture;
	*resets = guard_info.resets;
	*measured = guard_info.measured;
	*confirmed = guard_info.confirmed;
}

static void watchdog_feed(void)
{
#ifdef CONFIG_TLSR_SLOTS_SIM
	tlsr_slots_sim_watchdog(false);
#else
	sys_write8(TIMER_STATUS_WATCHDOG, TIMER_STATUS);
	sys_write32(0U, TIMER2_TICK);
#endif
}

/* The running image's identity: the CRC-32 stored at its end (not recomputed). */
static uint32_t own_tag(void)
{
	uint32_t slot = tlsr_slot_running();
	uint32_t size = 0U;
	uint32_t tag = 0U;

	(void)tlsr_slot_read(slot + TLSR_SLOT_SIZE_WORD, &size, sizeof(size));
	if ((size & 0xfU) == 4U && size >= 0x24U && size <= TLSR_SLOT_SIZE) {
		(void)tlsr_slot_read(slot + size - 4U, &tag, sizeof(tag));
	}
	/* 0xffffffff means "no tag" in erased flash. */
	return tag == 0xffffffffU ? 0U : tag;
}

static int counter_read(uint32_t tag, uint32_t *count, bool *confirmed)
{
	uint8_t buf[COUNTER_MAX];
	uint32_t stored;
	uint8_t c = 0xffU;
	int err = tlsr_slot_read(COUNTER_TAG, &stored, sizeof(stored));

	*count = 0U;
	*confirmed = false;
	if (err == 0 && stored != 0xffffffffU && stored != tag) {
		/* Boots counted, and a confirmation given, for another image. */
		return tlsr_slot_erase(COUNTER, TLSR_SECTOR);
	}
	if (err == 0) {
		err = tlsr_slot_read(COUNTER_CONFIRMED, &c, 1);
	}
	if (err == 0) {
		err = tlsr_slot_read(COUNTER_CNT, buf, sizeof(buf));
	}
	while (err == 0 && *count < sizeof(buf) && buf[*count] == 0x00U) {
		(*count)++;
	}
	*confirmed = err == 0 && stored == tag && c == 0x00U;
	return err;
}

static int counter_add(uint32_t tag, uint32_t count)
{
	static const uint8_t zero;
	uint32_t stored;
	int err = tlsr_slot_read(COUNTER_TAG, &stored, sizeof(stored));

	if (err == 0 && stored == 0xffffffffU) {
		err = tlsr_slot_write(COUNTER_TAG, &tag, sizeof(tag));
	}
	if (err == 0 && count < COUNTER_MAX) {
		err = tlsr_slot_write(COUNTER_CNT + count, &zero, 1);
	}
	return err;
}

/*
 * Erases the sector; with keep_confirmed, writes the image's tag and the
 * confirmed byte back. With keep_confirmed and the sector already holding this
 * image's tag, the confirmed byte and no counted boot, it does nothing: each
 * erase here holds interrupts off for up to 500 ms, wears the sector, and is
 * a window in which a power cut loses the confirmation.
 */
static int counter_clear(bool keep_confirmed)
{
	static const uint8_t confirmed = 0x00U;
	uint32_t tag;
	uint8_t first;
	uint8_t c = 0xffU;
	int err = tlsr_slot_read(COUNTER_TAG, &tag, sizeof(tag));

	if (err == 0) {
		err = tlsr_slot_read(COUNTER_CNT, &first, 1);
	}
	if (err == 0 && keep_confirmed) {
		err = tlsr_slot_read(COUNTER_CONFIRMED, &c, 1);
		if (err == 0 && c == 0x00U && first == 0xffU && tag == own_tag()) {
			return 0;
		}
	}
	if (err == 0 && (tag != 0xffffffffU || first != 0xffU)) {
		err = tlsr_slot_erase(COUNTER, TLSR_SECTOR);
	}
	if (err == 0 && keep_confirmed) {
		tag = own_tag();
		err = tlsr_slot_write(COUNTER_TAG, &tag, sizeof(tag));
		if (err == 0) {
			err = tlsr_slot_write(COUNTER_CONFIRMED, &confirmed, 1);
		}
	}
	return err;
}

static atomic_t watchdog_on;
static bool counted;    /* this boot was counted (by the early stage or the POST_KERNEL one) */

#ifdef CONFIG_TLSR_SLOTS_SIM
void tlsr_boot_guard_sim_new_boot(void)
{
	counted = false;
}
#endif

/*
 * Whether the image is confirmed, from the counter sector, not RAM: a revert
 * that stopped after it forgot the count (its writes did not take) leaves the
 * boot running with the sector erased, and neither the healthy mark nor a
 * planned wake may then act as if confirmed. Only once the boot
 * is running, and read here rather than by counter_read(), whose only caller
 * stays the early stage, so that stage compiles as proven.
 */
static bool confirmed_in_flash(void)
{
	uint32_t stored;
	uint8_t c;

	return tlsr_slot_read(COUNTER_TAG, &stored, sizeof(stored)) == 0 && stored == own_tag() &&
	       tlsr_slot_read(COUNTER_CONFIRMED, &c, 1) == 0 && c == 0x00U;
}

/* The automatic rule's healthy mark: only for an image the host has confirmed. */
static void healthy(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!confirmed_in_flash()) {
		guard_info.confirmed = false;
		LOG_INF("healthy, but the image is not confirmed: the count stays");
		return;
	}
	if (counter_clear(true) != 0) {
		LOG_ERR("could not clear the reset counter");
	} else {
		guard_info.resets = 0U;
	}
}

void tlsr_boot_guard_plan_wake(void)
{
	/* confirmed in RAM and in flash (as healthy() reads it); otherwise the wake is counted */
	if (guard_info.confirmed && confirmed_in_flash()) {
		tlsr_plan_reset(own_tag());
	}
}

int tlsr_boot_guard_confirm(void)
{
	int err = counter_clear(true);

	if (err != 0) {
		LOG_ERR("could not confirm: %d", err);
		return err;
	}
	guard_info.resets = 0U;
	guard_info.confirmed = true;
	LOG_INF("confirmed by the host: the automatic healthy rule applies from now on");
	return 0;
}

static K_WORK_DELAYABLE_DEFINE(healthy_work, healthy);

static void feeder(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (atomic_get(&watchdog_on) != 0) {
			watchdog_feed();
		}
		k_msleep(CONFIG_TLSR_BOOT_GUARD_WATCHDOG_MS / 4);
	}
}

K_THREAD_DEFINE(tlsr_wd_feeder, CONFIG_TLSR_BOOT_GUARD_FEEDER_STACK_SIZE, feeder, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO - CONFIG_TLSR_BOOT_GUARD_FEEDER_PRIO_ABOVE_LOWEST, 0, 0);

void tlsr_boot_guard_feed(void)
{
	/*
	 * Before the kernel, the SoC's early watchdog is running on the same
	 * Timer2 (CONFIG_SOC_TLSR8278_EARLY_WATCHDOG_MS): the early stage's
	 * revert (6 erases and 132 programs, up to 3.8 s by DS-TLSR8278 Table
	 * 19-13 against a 4 s period) feeds it the same way.
	 */
	if (atomic_get(&watchdog_on) != 0 || (!IS_ENABLED(CONFIG_TLSR_SLOTS_SIM) && k_is_pre_kernel())) {
		watchdog_feed();
	}
}

void tlsr_boot_guard_forget(void)
{
	/* The image is being left: its count and its confirmation go. */
	if (counter_clear(false) != 0) {
		LOG_ERR("could not clear the reset counter");
	}
}

/*
 * Whether this boot is one the firmware asked for, into this image (an update,
 * a revert, &prev_fw): the mark tlsr_reboot() leaves, read once per boot.
 */
static bool planned_boot(void)
{
	return tlsr_reboot_was_planned(own_tag());
}

/* Finishes an interrupted revert, counts this boot (unless planned), reverts after enough of them. */
static void count_boot(bool planned)
{
	uint32_t tag;
	uint32_t count;
	int err;

	counted = true;

	/* Before the first write (the POST_KERNEL stage without the early one). */
	if (guard_info.unlock == INT_MIN) {
		flash_unlock();
	}

	/* A revert that a reset cut short: finish it (reboots). */
	err = tlsr_slot_revert_resume();
	if (err < 0) {
		LOG_ERR("revert journal: %d", err);
	}

	tag = own_tag();
	err = counter_read(tag, &count, &guard_info.confirmed);
	/* Read (and clear) the flag even when the reset was planned. */
	const bool wd = reset_was_watchdog();

	/*
	 * A confirmed image counts no boot: its way back is the chord (read
	 * before this, or right after it with TLSR_BOOT_GUARD_CHORD_AFTER_COUNT)
	 * or &prev_fw, and no flash is written nor a revert started after a
	 * reset nobody asked for.
	 */
	if ((wd || IS_ENABLED(CONFIG_TLSR_BOOT_GUARD_COUNT_ALL_BOOTS)) && !guard_info.confirmed &&
	    !planned && err == 0) {
		err = counter_add(tag, count);
		count++;
		LOG_WRN("%s %u of %u without a healthy boot (%s)", wd ? "watchdog reset" : "boot",
			count, CONFIG_TLSR_BOOT_GUARD_RESETS,
			guard_info.confirmed ? "image confirmed" : "image not confirmed yet");
	}
	if (err != 0 && !planned) {
		/*
		 * The count could not be kept (a write that did not read back, a
		 * read error): the guard cannot promise to go back later, so it goes
		 * back now. If that fails too (the slots locked or the flash bad),
		 * the boot goes on with tlsr_slot_write_failed() set for `info`, and
		 * the chord and the host's commands remain. Not on a planned boot
		 * (an update, a revert, &prev_fw): two guard images with a counter
		 * sector that will not erase would otherwise revert to each other
		 * at every boot and never reach the kernel.
		 */
		LOG_ERR("reset counter: %d; going back to the other image now", err);
		(void)tlsr_slot_revert();
	} else if (err != 0) {
		LOG_ERR("reset counter: %d", err);
	} else if (count >= CONFIG_TLSR_BOOT_GUARD_RESETS) {
		/* Reboots into the other slot; returns only when that is not possible. */
		(void)tlsr_slot_revert();
	}
	guard_info.resets = (uint8_t)MIN(count, UINT8_MAX);
}

#if defined(CONFIG_TLSR_BOOT_GUARD_CHORD)
/*
 * A key chord held at power-on goes back to the other image, so that an
 * image whose USB does not work, or that no host tool can reach, can still
 * be left without a tool. Read here, before the kernel. With active-low
 * rows (the V75 Pro): the row precharged high then made an input with its
 * pull-up, the column driven low, the row read. With
 * active-high rows (the V21): the same with the levels swapped, the row on
 * its 100 k pull-down and the column driven high. Three reads 1 ms apart
 * must all show both keys down. The pins (from the kscan node's row-gpios
 * and col-gpios; the row's GPIO_ACTIVE_LOW flag gives the polarity) go back
 * to what they were; the kscan driver configures them later.
 */
#define CHORD_KSCAN DT_CHOSEN(zmk_kscan)
#define CHORD_ROW_BASE(n) DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(CHORD_KSCAN, row_gpios, n))
#define CHORD_ROW_PIN(n)  DT_GPIO_PIN_BY_IDX(CHORD_KSCAN, row_gpios, n)
#define CHORD_ACTIVE_LOW \
	((DT_GPIO_FLAGS_BY_IDX(CHORD_KSCAN, row_gpios, CONFIG_TLSR_BOOT_GUARD_CHORD_ROW) & \
	  GPIO_ACTIVE_LOW) != 0)
/* The two keys on two rows (the V21's Fn + KP-, the V75 Pro's Fn + Up), or on one. */
#define CHORD_TWO_ROWS (CONFIG_TLSR_BOOT_GUARD_CHORD_ROW_B != CONFIG_TLSR_BOOT_GUARD_CHORD_ROW)
BUILD_ASSERT(((DT_GPIO_FLAGS_BY_IDX(CHORD_KSCAN, row_gpios, CONFIG_TLSR_BOOT_GUARD_CHORD_ROW_B) &
	       GPIO_ACTIVE_LOW) != 0) == CHORD_ACTIVE_LOW,
	     "the chord's two rows must have the same polarity");
#define CHORD_COL_BASE(n) DT_REG_ADDR(DT_GPIO_CTLR_BY_IDX(CHORD_KSCAN, col_gpios, n))
#define CHORD_COL_PIN(n)  DT_GPIO_PIN_BY_IDX(CHORD_KSCAN, col_gpios, n)

/* The GPIO registers of a port (DS-TLSR8278 7.1): in, ie, oen, out, pol, ds, gpio-function. */
#define GPIO_IN(base)   ((base) + 0U)
#define GPIO_IE(base)   ((base) + 1U)
#define GPIO_OEN(base)  ((base) + 2U)
#define GPIO_OUT(base)  ((base) + 3U)
#define GPIO_FUNC(base) ((base) + 6U)
#define GPIO_GROUP(base) (((base) - 0x00800580U) / 8U)
/*
 * The afe_ registers are reached through 0x8000b8-0x8000ba, which DS-TLSR8278
 * does not document: the accessors below write the address, the data and the
 * control byte, and wait for bit 0 of the control byte to clear.
 */
#define ANALOG_PORT_ADDR  0x008000b8U
#define ANALOG_PORT_DATA  0x008000b9U
#define ANALOG_PORT_CTRL  0x008000baU
#define ANALOG_PORT_BUSY  BIT(0)
#define ANALOG_PORT_WRITE BIT(5)
#define ANALOG_PORT_START BIT(6)
#define ANA_PC_IE 0xc0U /* PC's input enable is an analog register on the TLSR8278 */
#define ANA_PULL_BASE 0x0eU /* two bits per pin: PA0-3 at 0x0e, PA4-7 at 0x0f, PB at 0x10-0x11, ... */
#define ANA_PULL_DOWN_100K 2U /* a pin's two bits: 0 none, 1 1 M up, 2 100 k down, 3 10 k up */

static uint8_t chord_analog_read(uint8_t addr)
{
	uint8_t v;

	sys_write8(addr, ANALOG_PORT_ADDR);
	sys_write8(ANALOG_PORT_START, ANALOG_PORT_CTRL);
	while ((sys_read8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	v = sys_read8(ANALOG_PORT_DATA);
	sys_write8(0U, ANALOG_PORT_CTRL);
	return v;
}

static void chord_analog_write(uint8_t addr, uint8_t value)
{
	sys_write8(addr, ANALOG_PORT_ADDR);
	sys_write8(value, ANALOG_PORT_DATA);
	sys_write8(ANALOG_PORT_START | ANALOG_PORT_WRITE, ANALOG_PORT_CTRL);
	while ((sys_read8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	sys_write8(0U, ANALOG_PORT_CTRL);
}

struct chord_pin {
	uint32_t base;
	uint8_t pin;
	uint8_t saved_oen, saved_out, saved_func;
};

static void chord_pin_save(struct chord_pin *p)
{
	p->saved_oen = sys_read8(GPIO_OEN(p->base));
	p->saved_out = sys_read8(GPIO_OUT(p->base));
	p->saved_func = sys_read8(GPIO_FUNC(p->base));
}

static void chord_pin_restore(const struct chord_pin *p)
{
	sys_write8(p->saved_oen, GPIO_OEN(p->base));
	sys_write8(p->saved_out, GPIO_OUT(p->base));
	sys_write8(p->saved_func, GPIO_FUNC(p->base));
}

/* A pin's output bit set to the level (true: high). */
static ALWAYS_INLINE void chord_out(const struct chord_pin *p, bool high)
{
	uint8_t b = BIT(p->pin);
	uint8_t v = sys_read8(GPIO_OUT(p->base));

	sys_write8(high ? (v | b) : (v & (uint8_t)~b), GPIO_OUT(p->base));
}

/* One column driven active, the row read: true when the key at (row, column) is down. */
static bool chord_key_down(const struct chord_pin *row, const struct chord_pin *col)
{
	const bool idle = CHORD_ACTIVE_LOW; /* the level of a released key's row and of an idle column */
	uint8_t rb = BIT(row->pin);
	bool down;

	/* The row precharged to its idle level, then an input on its pull. */
	chord_out(row, idle);
	sys_write8(sys_read8(GPIO_OEN(row->base)) & (uint8_t)~rb, GPIO_OEN(row->base));
	tlsr_spi_flash_delay_us(10);
	sys_write8(sys_read8(GPIO_OEN(row->base)) | rb, GPIO_OEN(row->base));
	/* This column driven active; the other one stays idle. */
	chord_out(col, !idle);
	tlsr_spi_flash_delay_us(1000);
	down = idle ? (sys_read8(GPIO_IN(row->base)) & rb) == 0U
		    : (sys_read8(GPIO_IN(row->base)) & rb) != 0U;
	chord_out(col, idle);
	return down;
}

struct chord_row {
	struct chord_pin pin;
	uint8_t pull_reg;
	bool is_pc;
	uint8_t saved_pull, saved_ie;
};

/* A row: saved, then GPIO function, its pull, its input enable. */
static void chord_row_setup(struct chord_row *r)
{
	const uint8_t rb = BIT(r->pin.pin);

	r->pull_reg = ANA_PULL_BASE + GPIO_GROUP(r->pin.base) * 2U + (r->pin.pin >= 4U ? 1U : 0U);
	r->is_pc = GPIO_GROUP(r->pin.base) == 2U;
	chord_pin_save(&r->pin);
	r->saved_pull = chord_analog_read(r->pull_reg);
	r->saved_ie = r->is_pc ? chord_analog_read(ANA_PC_IE) : sys_read8(GPIO_IE(r->pin.base));

	sys_write8(r->pin.saved_func | rb, GPIO_FUNC(r->pin.base));
	if (CHORD_ACTIVE_LOW) {
		/*
		 * The whole pull register (four pins) at 10 k up, the value the
		 * kscan driver sets later for the rows' port.
		 */
		chord_analog_write(r->pull_reg, r->saved_pull | 0xffU);
	} else {
		/*
		 * The row's own two bits at 100 k down; the register's other
		 * pins keep their pulls.
		 */
		const uint8_t shift = (r->pin.pin & 3U) * 2U;

		chord_analog_write(r->pull_reg, (r->saved_pull & (uint8_t)~(3U << shift)) |
							(uint8_t)(ANA_PULL_DOWN_100K << shift));
	}
	if (r->is_pc) {
		chord_analog_write(ANA_PC_IE, r->saved_ie | rb);
	} else {
		sys_write8(r->saved_ie | rb, GPIO_IE(r->pin.base));
	}
}

static void chord_row_restore(const struct chord_row *r)
{
	if (r->is_pc) {
		chord_analog_write(ANA_PC_IE, r->saved_ie);
	} else {
		sys_write8(r->saved_ie, GPIO_IE(r->pin.base));
	}
	chord_analog_write(r->pull_reg, r->saved_pull);
	chord_pin_restore(&r->pin);
}

static bool chord_held(void)
{
	/*
	 * Two rows that share a pull or input-enable register (the V21's PC2
	 * and PC1) are set up one after the other and restored in the reverse
	 * order, so each restore puts back the value its setup found.
	 */
	struct chord_row rows[2] = {
		{.pin = {CHORD_ROW_BASE(CONFIG_TLSR_BOOT_GUARD_CHORD_ROW),
			 CHORD_ROW_PIN(CONFIG_TLSR_BOOT_GUARD_CHORD_ROW)}},
		{.pin = {CHORD_ROW_BASE(CONFIG_TLSR_BOOT_GUARD_CHORD_ROW_B),
			 CHORD_ROW_PIN(CONFIG_TLSR_BOOT_GUARD_CHORD_ROW_B)}},
	};
	struct chord_row *row_b = CHORD_TWO_ROWS ? &rows[1] : &rows[0];
	struct chord_pin cols[2] = {
		{CHORD_COL_BASE(CONFIG_TLSR_BOOT_GUARD_CHORD_COL_A), CHORD_COL_PIN(CONFIG_TLSR_BOOT_GUARD_CHORD_COL_A)},
		{CHORD_COL_BASE(CONFIG_TLSR_BOOT_GUARD_CHORD_COL_B), CHORD_COL_PIN(CONFIG_TLSR_BOOT_GUARD_CHORD_COL_B)},
	};
	bool held = true;

	chord_row_setup(&rows[0]);
	if (CHORD_TWO_ROWS) {
		chord_row_setup(&rows[1]);
	}
	/*
	 * The columns: GPIO function, outputs driven idle (a column left
	 * floating would carry the other key's row active through a pressed
	 * switch: a ghost).
	 */
	chord_pin_save(&cols[0]);
	chord_pin_save(&cols[1]);
	for (int i = 0; i < 2; i++) {
		const uint8_t cb = BIT(cols[i].pin);

		sys_write8(sys_read8(GPIO_FUNC(cols[i].base)) | cb, GPIO_FUNC(cols[i].base));
		chord_out(&cols[i], CHORD_ACTIVE_LOW);
		sys_write8(sys_read8(GPIO_OEN(cols[i].base)) & (uint8_t)~cb, GPIO_OEN(cols[i].base));
	}

	for (int n = 0; n < 3 && held; n++) {
		held = chord_key_down(&rows[0].pin, &cols[0]) && chord_key_down(&row_b->pin, &cols[1]);
	}

	/* Everything back the way it was. */
	chord_pin_restore(&cols[1]);
	chord_pin_restore(&cols[0]);
	if (CHORD_TWO_ROWS) {
		chord_row_restore(&rows[1]);
	}
	chord_row_restore(&rows[0]);
	return held;
}
#endif /* CONFIG_TLSR_BOOT_GUARD_CHORD */

void tlsr_boot_guard_early(void)
{
	bool planned;

	/* Before any flash write of this boot, the chord's revert included. */
	flash_unlock();
#if defined(CONFIG_TLSR_BOOT_GUARD_CHORD)
	/* A revert that a reset cut short is finished first, as in count_boot(). */
	if (tlsr_slot_revert_resume() < 0) {
		LOG_ERR("revert journal");
	}
	planned = planned_boot();
	/*
	 * Not on a planned boot (an update, a revert, &prev_fw): with guard
	 * images in both slots and the chord held, each would revert to the
	 * other for as long as the keys are down.
	 */
	if (!IS_ENABLED(CONFIG_TLSR_BOOT_GUARD_CHORD_AFTER_COUNT) && !planned && chord_held()) {
		LOG_WRN("the chord is held at power-on: going back to the other image");
		(void)tlsr_slot_revert();
	}
#else
	planned = planned_boot();
#endif
	count_boot(planned);
#if defined(CONFIG_TLSR_BOOT_GUARD_CHORD_AFTER_COUNT)
	/* After the count: an unconfirmed image's hang in the chord's code is counted (Kconfig). */
	if (!planned && chord_held()) {
		LOG_WRN("the chord is held at power-on: going back to the other image");
		(void)tlsr_slot_revert();
	}
#endif
}

int tlsr_boot_guard_boot(void)
{
	if (!counted) {
		count_boot(planned_boot());
	}
	counted = false; /* the next boot counts again (a simulated one, on native_sim) */

	watchdog_setup();
	watchdog_start();
	atomic_set(&watchdog_on, 1);
	if (IS_ENABLED(CONFIG_TLSR_BOOT_GUARD_HEALTHY_UPTIME)) {
		(void)k_work_reschedule(&healthy_work, K_MSEC(CONFIG_TLSR_BOOT_GUARD_HEALTHY_MS));
	}
	return 0;
}

void tlsr_boot_guard_healthy_now(void)
{
	healthy(NULL);
}

void tlsr_boot_guard_healthy_soon(uint32_t delay_ms)
{
	(void)k_work_reschedule(&healthy_work, K_MSEC(delay_ms));
}

static int boot_guard_init(void)
{
	return tlsr_boot_guard_boot();
}

SYS_INIT(boot_guard_init, POST_KERNEL, CONFIG_TLSR_BOOT_GUARD_INIT_PRIORITY);

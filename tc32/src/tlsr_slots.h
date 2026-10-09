/*
 * Firmware slots of Telink TLSR827x keyboards with two images in flash,
 * at 0x00000 and TLSR_SLOT_B. The boot ROM starts the
 * slot whose byte 8 is 0x4b (the "KNLT" word at 8..11).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TLSR_SLOTS_H_
#define TLSR_SLOTS_H_

#include <stdint.h>
#include <zephyr/device.h>

#define TLSR_SLOT_A         0x00000U
#define TLSR_SLOT_B         CONFIG_TLSR_SLOT_B_OFFSET
#define TLSR_SLOT_SIZE      CONFIG_TLSR_SLOT_SIZE
#define TLSR_SLOT_FLAG      8U
#define TLSR_SLOT_FLAG_OK   0x4bU
#define TLSR_SLOT_SIZE_WORD 0x18U
#define TLSR_SECTOR         4096U
#define TLSR_STEP_CHUNK     10 /* tlsr_slots_sim_step(): after a sector-copy chunk */

/** @brief The flash device holding the slots. */
const struct device *tlsr_slot_flash(void);

/**
 * @brief Flash reads, writes and sector erases for the slots and the guard.
 *
 * Before the kernel runs they go straight to the flash routines of
 * tlsr_spi_flash.h (CONFIG_TLSR_SPI_FLASH); afterwards through the
 * flash driver.
 */
int tlsr_slot_read(uint32_t off, void *buf, size_t len);
int tlsr_slot_write(uint32_t off, const void *buf, size_t len);
int tlsr_slot_erase(uint32_t off, size_t len);

/** @brief The slot the running image was started from (register 0x63e). */
uint32_t tlsr_slot_running(void);

/**
 * @brief The running slot, cross-checked against flash before anything writes.
 *
 * The register alone decides which slot is running; if it were wrong, an
 * update or a revert would erase the running image. So the slot must be
 * marked bootable (byte 8 = 0x4b) and hold what the CPU executes: 512 bytes
 * from 0x100, read through XIP and through the flash controller, must match.
 *
 * @param slot The running slot.
 *
 * @retval 0 Consistent.
 * @retval -EIO The register and the flash disagree; nothing may be written.
 */
int tlsr_slot_running_checked(uint32_t *slot);

/** @brief The slot the running image was not started from. */
static inline uint32_t tlsr_slot_other(void)
{
	return tlsr_slot_running() == TLSR_SLOT_A ? TLSR_SLOT_B : TLSR_SLOT_A;
}

/**
 * @brief Check the image in a slot.
 *
 * The size word at 0x18 must be 16n + 4 and fit the slot, and the last four
 * bytes must be the CRC-32 of the rest, taking bytes 8..11 as "KNLT" (they are
 * cleared in a slot that is no longer bootable).
 *
 * @param slot Slot address.
 * @param crc  The image's CRC-32, or NULL.
 *
 * @retval 0 The slot holds a complete image.
 * @retval -ENOENT No valid image.
 * @retval <0 Flash error.
 */
int tlsr_slot_check(uint32_t slot, uint32_t *crc);

/**
 * @brief Make the other slot's image the one that boots, and reboot.
 *
 * Restores the other slot's "KNLT" word, then clears the running slot's. The
 * first sector of the other slot is rewritten from a copy kept in a journal
 * (CONFIG_TLSR_SLOTS_JOURNAL_OFFSET), so a power cut does not lose it:
 * tlsr_slot_revert_resume() finishes the job at the next boot.
 *
 * @retval -ENOENT The other slot has no valid image (nothing was changed), or its first
 *         sector as copied or rewritten does not check (a read that went wrong): then
 *         either nothing was changed, or the journal is kept, the other slot is not
 *         bootable and this one still is; the next boot copies the sector again.
 * @retval -EIO A flash write or its read-back did not match (the journal record, the other
 *         slot's flag, a sector), the running slot could not be confirmed, or the other image
 *         checks but is not the one the revert set out to boot. A flag that does not read back
 *         once its byte 8 holds 0x4b leaves both slots bootable, each image checked.
 * @retval <0 Flash error.
 * Does not return on success (on native_sim test builds it returns 0).
 */
int tlsr_slot_revert(void);

/**
 * @brief Finish a revert that a reset interrupted.
 *
 * A journal record that does not match the slots is read and compared once
 * more (tlsr_slot_drop_journal()) before it is dropped.
 *
 * @retval 0 No revert to finish, a stale journal was dropped, or the record
 *         read differently the second time and is kept for the next boot.
 * @retval -EIO The running slot is not confirmed (tlsr_slot_running_checked()),
 *         or the revert stopped: a write or read-back mismatch, or the other
 *         image checks but not with the recorded CRC.
 * @retval -ENOENT The revert stopped: the other image does not check.
 * @retval <0 Flash error.
 * Does not return when it finishes a revert (on native_sim test builds it
 * returns 1).
 */
int tlsr_slot_revert_resume(void);

/**
 * @brief Reset the chip into the image tagged @p tag (the CRC-32 word at its
 * end), marking the reset as planned for that image: its boot guard does not
 * count the boot or read the power-on chord. The mark is kept by a retained
 * analog register (0x3c), not RAM, so it does not depend on either build's
 * memory layout.
 */
void tlsr_reboot(uint32_t tag);

/**
 * @brief Mark the next reset as planned for the image tagged @p tag, as
 * tlsr_reboot() does, without resetting: for a reset that comes later on
 * its own (a wake from deep sleep, which keeps the register).
 */
void tlsr_plan_reset(uint32_t tag);

/**
 * @brief Whether this boot is a reset planned for the image tagged @p tag;
 * clears the mark. Called once per boot by the boot guard.
 */
bool tlsr_reboot_was_planned(uint32_t tag);

/** What the boot guard found in the mark register at this boot. */
enum tlsr_boot_mark {
	TLSR_BOOT_MARK_NONE,    /* the power-on value: a power-on, or a reset nothing marked */
	TLSR_BOOT_MARK_PLANNED, /* this image's mark: a reboot the firmware asked for, into it */
	TLSR_BOOT_MARK_OTHER,   /* another image's mark, kept through the resets since it was left */
};

/** @brief What tlsr_reboot_was_planned() found at this boot (the version reply reports it). */
enum tlsr_boot_mark tlsr_boot_mark(void);

/**
 * @brief Whether the mark register did not read back its cleared value at this
 * boot; the boot was then taken as unplanned (the version reply reports it).
 */
bool tlsr_boot_mark_stuck(void);

/** @brief The byte a planned reset into the image tagged @p tag leaves as the mark. */
uint8_t tlsr_planned_mark(uint32_t tag);

/**
 * @brief A number that changes at each tlsr_slot_touched(): after the OTA
 * receiver's writes and erases of an update, and after a running image's
 * revert (&prev_fw) that returned. A check of the other slot made before it
 * changed may be stale. The writes of tlsr_slots.c do not change it: they
 * also run before the boot is counted, which stays as proven, and the
 * version reply checks anew at every boot.
 */
uint32_t tlsr_slot_generation(void);

/** @brief A write or erase of a slot, or a revert that stopped: see tlsr_slot_generation(). */
void tlsr_slot_touched(void);

/** @brief Feed the boot guard's watchdog during long flash work (no-op without it). */
void tlsr_boot_guard_feed(void);

/** @brief Forget counted watchdog resets (a revert has started). */
void tlsr_boot_guard_forget(void);

/**
 * @brief Whether a tlsr_slot_write() or tlsr_slot_erase() of this boot did not
 * read back (a locked or failing flash): the boot guard's count, the confirm,
 * the revert's journal and slot writes. Stays set for the boot.
 */
bool tlsr_slot_write_failed(void);

/**
 * @brief The flash status register as the boot guard found it and left it
 * (low byte | high byte << 8; 0 when the guard did not read it), and flags:
 * bit 0 the guard cleared the block protection this boot, bit 1 the bits are
 * still set after its write, bit 2 a guard or revert flash write did not read
 * back (tlsr_slot_write_failed()), bit 3 the part is not one the unlock
 * handles (nothing written), bit 4 the register was read this boot (else the
 * values are 0 because nothing read them), bit 5 the register read as busy or
 * write-enabled (a misread), so nothing was written. *mid is the flash's
 * JEDEC ID as tlsr_spi_flash_jedec_id() composes it (0 when not read).
 */
void tlsr_boot_guard_flash_info(uint16_t *status_boot, uint16_t *status_now, uint8_t *flags,
				uint32_t *mid);

/**
 * @brief The boot guard's boot step (run at POST_KERNEL; exposed for tests).
 *
 * Finishes an interrupted revert, counts the boot (every unplanned one, or
 * only one after a watchdog reset), reverts after CONFIG_TLSR_BOOT_GUARD_RESETS
 * of them, and starts the watchdog.
 *
 * @retval 0 Always; problems are logged.
 */
int tlsr_boot_guard_boot(void);

/**
 * @brief The boot guard's early stage: from the board's early-init hook,
 * before any device or thread (CONFIG_TLSR_BOOT_GUARD_EARLY).
 *
 * Finishes an interrupted revert, counts the boot, and reverts after
 * CONFIG_TLSR_BOOT_GUARD_RESETS of them, through tlsr_spi_flash.h;
 * tlsr_boot_guard_boot() then only starts the watchdog and the healthy rule.
 */
void tlsr_boot_guard_early(void);

/**
 * @brief Before a deep sleep: a confirmed image (in RAM and in the counter
 * sector) marks the wake as a planned reset (not counted, no power-on
 * chord), so that wakes with no host in reach cannot add up to a revert; an
 * unconfirmed image leaves it counted.
 */
void tlsr_boot_guard_plan_wake(void);

/** @brief The automatic healthy rule's mark: clears the count if the image is confirmed. */
void tlsr_boot_guard_healthy_now(void);

/** @brief The same from any context, delay_ms from now: the clearing runs on the system work queue. */
void tlsr_boot_guard_healthy_soon(uint32_t delay_ms);

/**
 * @brief What the boot guard set up at this boot.
 *
 * @param sclk_hz  System clock used for the watchdog period (measured, or the
 *                 CONFIG_TLSR_BOOT_GUARD_SCLK_HZ fallback).
 * @param capture  Timer2 watchdog capture (period = capture * 2^18 / sclk_hz).
 * @param resets   Watchdog resets counted at this boot.
 * @param measured Whether sclk_hz was measured.
 */
void tlsr_boot_guard_info(uint32_t *sclk_hz, uint16_t *capture, uint8_t *resets, bool *measured,
			  bool *confirmed);

/**
 * @brief The host confirms the running image (the OTA report's 0xff03).
 *
 * tlsr_usb_ota.c calls it only after the flash test passed in the same boot and
 * while the other slot's image checks when the confirm arrives.
 *
 * Clears the boot count and writes the confirmed byte under the image's tag,
 * which stays until the counter is dropped for another image; from then on
 * the automatic healthy rule clears the count.
 *
 * @retval 0 on success, else a flash error.
 */
int tlsr_boot_guard_confirm(void);

#ifdef CONFIG_TLSR_SLOTS_SIM
/* Test hooks, provided by the test. */
uint32_t tlsr_slots_sim_running_slot(void);
void tlsr_slots_sim_reboot(void);
bool tlsr_slots_sim_reset_was_watchdog(void);
void tlsr_slots_sim_watchdog(bool start);
/*
 * Called before each revert step (1-6) and after each 64-byte chunk of a
 * sector copy (TLSR_STEP_CHUNK); nonzero stops there, as a power cut would.
 */
int tlsr_slots_sim_step(int step);
/* What the CPU sees at flash offset off (XIP): the slot it really runs from. */
int tlsr_slots_sim_cpu_view(uint32_t off, uint8_t *buf, size_t len);
/* After each read of tlsr_slot_read(), with the bytes read: a test can change them (a read error). */
void tlsr_slots_sim_read_done(uint32_t off, void *buf, size_t len);
/* The system clock the boot guard measures (0 or out of range: a failed measurement). */
uint32_t tlsr_slots_sim_sclk_hz(void);
/*
 * A locked (or failing) flash: true drops the write or erase of those bytes,
 * so that it does not read back. Weak; false when the test does not define it.
 */
bool tlsr_slots_sim_protected(uint32_t off, size_t len);
/* Forgets the boot's write-failed flag, as a new boot would (the test's setup). */
void tlsr_slots_sim_new_boot(void);
/* Gives the guard flash values to report (the version reply's bytes 19-28), as the unlock would have. */
void tlsr_boot_guard_sim_flash(uint16_t status_boot, uint16_t status_now, int unlock, uint32_t mid);
/* Forgets that the boot was counted, as a new boot would (the test's setup). */
void tlsr_boot_guard_sim_new_boot(void);
/*
 * The retained register of the planned-reboot mark (analog 0x3c on the chip):
 * kept across the simulated reboots, 0x0f at power-on. Weak; a variable in
 * tlsr_slots.c when the test does not define them.
 */
uint8_t tlsr_slots_sim_retained_read(void);
void tlsr_slots_sim_retained_write(uint8_t value);
#endif

#endif /* TLSR_SLOTS_H_ */

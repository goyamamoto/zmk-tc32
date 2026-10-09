/*
 * OTA part of the scripted USB host (the Telink USB OTA receiver of tc32/).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TLSR_USB_OTA_HOST_H_
#define TLSR_USB_OTA_HOST_H_

#include <stdint.h>

/** A HID interface found in the configuration descriptor. */
struct tlsr_hid_info {
	uint8_t intf;
	uint16_t report_len;
	uint8_t in_ep;
	uint16_t in_mps;
};

/**
 * @brief Update both firmware slots through the OTA interface and check flash.
 *
 * @param hid The HID interface with output report 5.
 *
 * @return Number of failed checks.
 */
int tlsr_usb_ota_host_test(const struct tlsr_hid_info *hid);

/**
 * @brief Check the &prev_fw key (held 1 s, then 3.5 s by the mock matrix).
 *
 * Runs after tlsr_usb_ota_host_test(), which leaves image 2 in slot A and
 * image 1, no longer bootable, in slot B.
 *
 * @return Number of failed checks.
 */
int tlsr_prev_fw_host_test(void);

/** @brief Version command on the OTA interface; returns failed checks. */
int tlsr_usb_ota_host_version(const struct tlsr_hid_info *hid);

/* The confirm command: the count is 1 before, the reply and info show 0 after. */
int tlsr_usb_ota_host_confirm(const struct tlsr_hid_info *hid);

int tlsr_usb_ota_host_test_edges(const struct tlsr_hid_info *hid);

/* The gate: an update over the other slot's image only after the unlock (0xff05). */
int tlsr_usb_ota_host_gate(const struct tlsr_hid_info *hid);

#endif /* TLSR_USB_OTA_HOST_H_ */

/*
 * Stand-in for the host test: the flash driver calls go to the test's flash
 * model (test_rng_store.c).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
struct device {
	int unused;
};
extern const struct device test_flash_dev;
#define DEVICE_DT_GET(node) (&test_flash_dev)
int flash_read(const struct device *dev, off_t offset, void *data, size_t len);
int flash_write(const struct device *dev, off_t offset, const void *data, size_t len);
int flash_erase(const struct device *dev, off_t offset, size_t size);

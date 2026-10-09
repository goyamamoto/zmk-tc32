/*
 * Zephyr flash driver over the flash routines of tlsr_spi_flash.c, so that
 * the slot code, the OTA receiver and the boot guard keep the Zephyr flash API
 * while every SPI transaction goes through tlsr_spi_flash_io.S.
 *
 * With the own BLE stack (TLSR_BLE) each write and each sector erase holds
 * the radio off for its whole length first (ble_flash_radio_hold(): no
 * connection or advertising event on air). The hold is here, not in
 * tlsr_spi_flash.c, whose page program
 * and erase also run before the boot is counted (the boot guard's early
 * stage, code that stays as proven on the unit); there, before the kernel,
 * no radio runs. After boot every write goes through this driver: the slot
 * code (tlsr_slots.c writes directly only before the kernel), the OTA
 * receiver, the boot guard's later writes, the BLE bonds, the backlight
 * settings.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#define DT_DRV_COMPAT telink_tlsr_spi_flash

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>

#include "tlsr_spi_flash.h"
#if IS_ENABLED(CONFIG_TLSR_BLE)
#include "ble/ble_flash.h"
#else
static inline void ble_flash_radio_hold(bool hold)
{
	ARG_UNUSED(hold);
}
#endif

struct spi_flash_config {
	uint32_t base;
	size_t size;
};

struct spi_flash_data {
	struct k_sem lock;
	struct flash_pages_layout layout;
};

static const struct flash_parameters spi_flash_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

static bool range_valid(const struct spi_flash_config *config, off_t offset, size_t len)
{
	return offset >= 0 && (size_t)offset <= config->size && len <= config->size - (size_t)offset;
}

static int spi_flash_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct spi_flash_config *config = dev->config;
	struct spi_flash_data *dd = dev->data;

	if (!range_valid(config, offset, len)) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}
	k_sem_take(&dd->lock, K_FOREVER);
	tlsr_spi_flash_read(config->base + (uint32_t)offset, len, data);
	k_sem_give(&dd->lock);
	return 0;
}

static int spi_flash_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct spi_flash_config *config = dev->config;
	struct spi_flash_data *dd = dev->data;

	if (!range_valid(config, offset, len)) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}
	k_sem_take(&dd->lock, K_FOREVER);
	ble_flash_radio_hold(true);
	tlsr_spi_flash_page_program(config->base + (uint32_t)offset, len, data);
	ble_flash_radio_hold(false);
	k_sem_give(&dd->lock);
	return 0;
}

static int spi_flash_erase(const struct device *dev, off_t offset, size_t len)
{
	const struct spi_flash_config *config = dev->config;
	struct spi_flash_data *dd = dev->data;

	if (!range_valid(config, offset, len) || (offset % TLSR_SPI_FLASH_SECTOR) != 0 ||
	    (len % TLSR_SPI_FLASH_SECTOR) != 0U) {
		return -EINVAL;
	}
	k_sem_take(&dd->lock, K_FOREVER);
	for (size_t off = 0; off < len; off += TLSR_SPI_FLASH_SECTOR) {
		/* a sector at a time: a connection event may come between two */
		ble_flash_radio_hold(true);
		tlsr_spi_flash_erase_sector(config->base + (uint32_t)offset + off);
		ble_flash_radio_hold(false);
	}
	k_sem_give(&dd->lock);
	return 0;
}

static const struct flash_parameters *spi_flash_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &spi_flash_parameters;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void spi_flash_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
				  size_t *layout_size)
{
	struct spi_flash_data *dd = dev->data;

	*layout = &dd->layout;
	*layout_size = 1U;
}
#endif

static int spi_flash_init(const struct device *dev)
{
	const struct spi_flash_config *config = dev->config;
	struct spi_flash_data *dd = dev->data;

	k_sem_init(&dd->lock, 1, 1);
	dd->layout.pages_count = config->size / TLSR_SPI_FLASH_SECTOR;
	dd->layout.pages_size = TLSR_SPI_FLASH_SECTOR;
	/* The calibration bytes applied once per boot; done already if the early boot guard ran. */
	tlsr_spi_flash_trim_calib();
	return 0;
}

static DEVICE_API(flash, spi_flash_api) = {
	.read = spi_flash_read,
	.write = spi_flash_write,
	.erase = spi_flash_erase,
	.get_parameters = spi_flash_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = spi_flash_page_layout,
#endif
};

#define SPI_FLASH_INIT(n)                                                                          \
	static const struct spi_flash_config spi_flash_config_##n = {                              \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
	};                                                                                         \
	static struct spi_flash_data spi_flash_data_##n;                                           \
	DEVICE_DT_INST_DEFINE(n, spi_flash_init, NULL, &spi_flash_data_##n,                        \
			      &spi_flash_config_##n, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,      \
			      &spi_flash_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_FLASH_INIT)

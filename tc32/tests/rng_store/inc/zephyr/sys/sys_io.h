/*
 * Stand-in for the host test: register accesses go to the test's model
 * (test_rng_store.c).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once
#include <stdint.h>
uint32_t test_reg_read(uint32_t addr, int size);
void test_reg_write(uint32_t addr, uint32_t value, int size);
#define sys_read8(a)     ((uint8_t)test_reg_read((a), 1))
#define sys_read32(a)    test_reg_read((a), 4)
#define sys_write8(v, a)  test_reg_write((a), (v), 1)
#define sys_write32(v, a) test_reg_write((a), (v), 4)

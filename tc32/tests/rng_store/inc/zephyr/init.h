/*
 * Stand-in for the host test: SYS_INIT() names the init function only.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once
#define SYS_INIT(fn, level, prio) int (*const test_sys_init_##fn)(void) = fn

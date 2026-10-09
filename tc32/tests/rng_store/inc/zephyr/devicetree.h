/*
 * Stand-in for the host test: the two nodes the files under test name.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once
#define DT_NODELABEL(label)                      DTNL_##label
#define DT_INST(inst, compat)                    DTINST_##compat
#define Z_DT_CAT2(a, b)                          a##b
#define Z_DT_CAT(a, b)                           Z_DT_CAT2(a, b)
#define DT_REG_ADDR(node)                        Z_DT_CAT(node, _ADDR)
#define DT_REG_SIZE(node)                        Z_DT_CAT(node, _SIZE)
#define DT_HAS_CHOSEN(x)                         0
#define DT_CHOSEN(x)                             test_flash_node
#define DT_HAS_COMPAT_STATUS_OKAY(compat)        1
#define DTNL_rng_seed_partition_ADDR             0x6b000U
#define DTNL_rng_seed_partition_SIZE             0x2000U
#define DTINST_telink_b87_rbg_ADDR          0x00804400U

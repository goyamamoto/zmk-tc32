#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Builds and runs the host test of src/ble/ble_crypto.c (tc32_aes128_encrypt()
# from soft_aes.c). Needs only a host C compiler: the few Zephyr helpers the
# file uses get plain stand-ins.
set -eu
D=$(cd "$(dirname "$0")" && pwd)
B=${TMPDIR:-/tmp}/ble_crypto_test.$$
mkdir -p "$B"
mkdir -p "$B/inc/zephyr/sys" "$B/inc/zephyr/crypto"
printf '#define BIT(n) (1UL << (n))\n' > "$B/inc/zephyr/sys/util.h"
printf '#include <stdint.h>\nvoid tc32_aes128_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);\n' > "$B/inc/zephyr/crypto/tc32_aes.h"
cat > "$B/inc/zephyr/sys/byteorder.h" <<'H'
#include <stdint.h>
static inline uint32_t sys_get_le32(const uint8_t *p) { return p[0] | p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static inline void sys_put_le32(uint32_t v, uint8_t *p) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static inline void sys_put_be16(uint16_t v, uint8_t *p) { p[0] = v >> 8; p[1] = v; }
H
cc -std=c11 -Wall -Wno-unused-function -I"$B/inc" -I"$D/../../src/ble" -include "$D/../../src/ble/ble_crypto.h" \
  -o "$B/t" "$D/test_ble_crypto.c" "$D/../../src/ble/ble_crypto.c" "$D/soft_aes.c"
"$B/t"
rc=$?
rm -rf "$B"
exit $rc

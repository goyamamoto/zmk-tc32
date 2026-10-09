#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Builds and runs the host test of src/tlsr_rng_store.c (test_rng_store.c) with zephyr-tc32's tc32_rng: its
# subsys/tc32_rng/tc32_rng.c and include/ from $ZEPHYR_BASE, by default the workspace's zephyr/ (tc32/scripts/
# build.sh), with the keyboards' Kconfig values and TC32_RNG_TEST; tc32_aes128_encrypt() from
# tests/ble_crypto/soft_aes.c. Needs only a host C compiler. Exit status is the test result.
set -eu
D=$(cd "$(dirname "$0")" && pwd)
R=$(cd "$D/../../.." && pwd)
Z=${ZEPHYR_BASE:-$(cd "${TC32_WORKSPACE:-$R/..}" && pwd)/zephyr}
B=${TMPDIR:-/tmp}/rng_store_test.$$
mkdir -p "$B"
trap 'rm -rf "$B"' EXIT
# The store as built, its two hooks renamed so that the test can watch them, and a reset for each boot.
sed -e 's/^int tc32_rng_seed_load(/int store_seed_load(/' -e 's/^int tc32_rng_seed_store(/int store_seed_store(/' \
  "$R/tc32/src/tlsr_rng_store.c" > "$B/tlsr_rng_store.c"
printf 'void store_reset(void);\nvoid store_reset(void)\n{\n\tcur = UNKNOWN;\n\tnext = 0U;\n\tseq = 0U;\n}\n' \
  >> "$B/tlsr_rng_store.c"
# The SPI flash transport's read: the test's (the device ID).
cat > "$B/tlsr_spi_flash_io.h" <<'H'
typedef void (*tlsr_spi_flash_io_read_t)(unsigned char cmd, unsigned long addr, unsigned char with_addr,
					 unsigned char zeros, unsigned char *buf, unsigned long len);
void test_flash_io_read(unsigned char cmd, unsigned long addr, unsigned char with_addr, unsigned char zeros,
			unsigned char *buf, unsigned long len);
#define TLSR_SPI_FLASH_IO_OFF_READ_CMD 0
#define TLSR_SPI_FLASH_IO_FN(type, off) ((type)test_flash_io_read)
H
cc -std=c11 -O1 -Wall -Wno-unused-function -Wno-unused-variable -I"$D/inc" -I"$Z/include" \
  -DCONFIG_TC32_RNG_JITTER_SAMPLES_PER_BIT=16 -DCONFIG_TC32_RNG_JITTER_CHUNK=256 -DCONFIG_TC32_RNG_RESEED_JITTER=16 \
  -DCONFIG_TC32_RNG_RESEED_INTERVAL=57 -DCONFIG_TC32_RNG_TRNG_BYTES=16 -DCONFIG_TC32_RNG_EVENT_CREDIT=0 \
  -DCONFIG_TC32_RNG_INIT_PRIORITY=90 -DCONFIG_TC32_RNG_COLLECT_TIMEOUT_MS=250 -DCONFIG_TC32_RNG_TEST=1 \
  -o "$B/t" "$D/test_rng_store.c" "$B/tlsr_rng_store.c" "$Z/subsys/tc32_rng/tc32_rng.c" "$D/../ble_crypto/soft_aes.c"
"$B/t"

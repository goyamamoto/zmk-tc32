/*
 * Raw samples of tc32_rng's sources over the update receiver
 * (src/tlsr_rng_measure.c, TLSR_RNG_MEASURE: measurement images only).
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef TC32_TLSR_RNG_MEASURE_H_
#define TC32_TLSR_RNG_MEASURE_H_

#include <stdint.h>

/*
 * The command 0xff0e: turns the request r into the reply (the receiver's
 * 33-octet report). -EINVAL for a request it does not know.
 */
int tlsr_rng_measure(uint8_t r[33]);

#endif /* TC32_TLSR_RNG_MEASURE_H_ */

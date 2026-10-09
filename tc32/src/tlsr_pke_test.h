/*
 * The P-256 engine's known answers for a host tool (CONFIG_TLSR_PKE_TEST).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TC32_TLSR_PKE_TEST_H_
#define TC32_TLSR_PKE_TEST_H_

#include <stddef.h>
#include <stdint.h>

/*
 * n octets of the record from offset at; the number copied. A request from
 * offset 0 runs the operations first. The record: octet 0 the results as
 * bits (0: the debug private key times G is the debug public key, 1: the
 * recorded responder's private key times G is its public key, 2: that key
 * times the recorded initiator's public key is their DHKey, 3: the debug
 * public key is on the curve, 4: a point off the curve is refused, 5: a
 * scalar of 0 is refused); octets 1-6 each operation's return code; 8-27
 * the time of the first five in us, 32-bit each; 28-59 and 60-91 the x and
 * y the first gave, 92-123 and 124-155 those of the second, 156-187 the x
 * the third gave.
 */
size_t tlsr_pke_test_read(size_t at, uint8_t *out, size_t n);

#define TLSR_PKE_TEST_RECORD 188U

#endif /* TC32_TLSR_PKE_TEST_H_ */

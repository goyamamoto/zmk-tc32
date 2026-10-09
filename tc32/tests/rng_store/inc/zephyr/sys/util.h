/*
 * Stand-in for the host test: the macros the files under test use, IS_ENABLED()
 * and IF_ENABLED() as Zephyr defines them.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define BIT(n)            (1UL << (n))
#define MIN(a, b)         (((a) < (b)) ? (a) : (b))
#define ARRAY_SIZE(a)     (sizeof(a) / sizeof((a)[0]))
#define ARG_UNUSED(x)     (void)(x)
#define BUILD_ASSERT(c, ...) _Static_assert(c, "" __VA_ARGS__)
#undef __weak
#define __weak            __attribute__((weak))
#define __noinline        __attribute__((noinline))
#define _XXXX1 _YYYY,
#define Z_IS_ENABLED3(ignore_this, val, ...) val
#define Z_IS_ENABLED2(one_or_two_args)       Z_IS_ENABLED3(one_or_two_args 1, 0)
#define Z_IS_ENABLED1(config_macro)          Z_IS_ENABLED2(_XXXX##config_macro)
#define IS_ENABLED(config_macro)             Z_IS_ENABLED1(config_macro)
#define Z_DEBRACKET(...)                     __VA_ARGS__
#define Z_GET_ARG2_DEBRACKET(ignore_this, val, ...) Z_DEBRACKET val
#define Z_COND_CODE(one_or_two_args, _if_code, _else_code)                                         \
	Z_GET_ARG2_DEBRACKET(one_or_two_args _if_code, _else_code)
#define Z_COND_CODE_1(_flag, _if_1_code, _else_code) Z_COND_CODE(_XXXX##_flag, _if_1_code, _else_code)
#define COND_CODE_1(_flag, _if_1_code, _else_code)  Z_COND_CODE_1(_flag, _if_1_code, _else_code)
#define IF_ENABLED(_flag, _code)                    COND_CODE_1(_flag, _code, ())

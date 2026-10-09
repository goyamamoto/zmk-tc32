/*
 * The TLSR_HOT_TEXT_PAD bytes a board's hot_text.ld places after its hot text, as data: 0xd4 each,
 * the bytes of the Thumb trap fill 0xd4d4, under a data mapping symbol of their own. Both build
 * paths then write them as they are (thumb2tc32.py leaves data alone; on the direct path they are
 * data to the linker), whatever precedes them. Never executed.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
	.syntax unified
	.thumb
	.section .tlsr_hot_text_pad, "ax", %progbits
	@ The assembler gives a section holding only data no mapping symbol; this label is one.
$d:
	.space CONFIG_TLSR_HOT_TEXT_PAD, 0xd4

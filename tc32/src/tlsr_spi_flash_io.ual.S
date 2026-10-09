/*
 * The SPI flash transport: sending a command and an address to the flash,
 * reading and writing through 0x80000c/0x80000d, waiting for a program or
 * erase to finish, and the microsecond delay they use. Its 496 bytes must
 * equal tlsr_spi_flash_io.json's blob_sha256, which every build checks, on
 * both paths (tc32-devtools compiler/image_check.py --blob).
 *
 * Registers:
 * - 0x80000c and 0x80000d (not documented). The code writes a command,
 *   address, data or 0 byte to 0x80000c and then waits until 0x80000d bit 4
 *   reads 0; it writes 1 to 0x80000d before a command (for 1 us) and after
 *   the last byte of a transaction, and 0 before the command byte; before
 *   reading data it writes 0x0a to 0x80000d and then reads each data byte
 *   from 0x80000c.
 * - 0x800643 (bit 0 the interrupt enable, bit 1 reserved): the whole byte is
 *   saved, set to 0 for a read or write transaction and written back. The
 *   code is in RAM (.ram_code).
 * - 0x800740, the 32-bit system timer, 16 ticks per microsecond.
 * The flash commands are the flash chip's: 0x05 (read status register; bit
 * 0 = 1 while a program or erase is in progress) and 0x06 (write enable).
 *
 * tst rn, rm sets the flags from rn & rm and writes no register.
 *
 * The C side calls the functions through offsets into tlsr_spi_flash_io
 * (tlsr_spi_flash_io.h, TLSR_SPI_FLASH_IO_OFF_*), so their places are fixed:
 * send_addr 0x000, send_cmd 0x040, wait_ready 0x06c, read_cmd 0x0b0,
 * write_cmd 0x160, sleep_us 0x1dc. The halfword 0x46c0 before a literal pool
 * pads it to a word boundary and is never executed.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * The standard-syntax twin of tlsr_spi_flash_io.S: the same code and data in
 * unified Thumb syntax, the TC32-only instructions as .inst.n with the Thumb
 * value that stands for each (the Telink instruction in the comment). The
 * direct path (TC32_DIRECT) assembles this file with clang -mcpu=tc32; the
 * Thumb path assembles tlsr_spi_flash_io.S through tc32asm2thumb.py. Every
 * build assembles both with the same flags and compares the objects
 * (asm_twin_check.py).
 */
	.syntax unified
	.thumb

	.section .ram_code.tlsr_spi_flash_io, "ax", %progbits
	.balign 4
	.global tlsr_spi_flash_io
	@ A plain label, neither function nor object: the C side adds the offsets
	@ and bit 0 itself, so its value must be the even start address (a Thumb
	@ function symbol would carry bit 0), and thumb2tc32.py leaves a sized
	@ object as data, which this is not.
tlsr_spi_flash_io:

/*
 * send_addr(addr in r0): the 24-bit flash address to 0x80000c, high byte
 * first, each byte followed by the wait for 0x80000d bit 4 = 0. 0x80000d is
 * not written (send_cmd before it wrote 0).
 */
	.type tlsr_flash_send_addr, %function
tlsr_flash_send_addr:
	lsls r2, r0, #8	@ bits 23:16 of the address ...
	lsrs r2, r2, #24	@ ... alone in r2
	ldr r3, .Lsa_data
	strb r2, [r3]	@ to 0x80000c
	ldr r1, .Lsa_ctrl
	movs r2, #0x10	@ 0x80000d bit 4
1:	ldrb r3, [r1]
	tst r2, r3
	bne 1b	@ until 0x80000d bit 4 reads 0
	lsls r2, r0, #16	@ bits 15:8
	lsrs r2, r2, #24
	ldr r3, .Lsa_data
	strb r2, [r3]
	ldr r1, .Lsa_ctrl
	movs r2, #0x10
1:	ldrb r3, [r1]
	tst r2, r3
	bne 1b
	lsls r0, r0, #24	@ bits 7:0
	lsrs r0, r0, #24
	ldr r3, .Lsa_data
	strb r0, [r3]
	ldr r1, .Lsa_ctrl
	movs r2, #0x10
1:	ldrb r3, [r1]
	tst r2, r3
	bne 1b
	bx lr
.Lsa_data:	.word	0x0080000c
.Lsa_ctrl:	.word	0x0080000d
	.size tlsr_flash_send_addr, . - tlsr_flash_send_addr

/*
 * send_cmd(cmd in r0): 0x80000d = 1, sleep_us(1), 0x80000d = 0, the command
 * byte to 0x80000c, the wait for 0x80000d bit 4 = 0.
 */
	.type tlsr_flash_send_cmd, %function
tlsr_flash_send_cmd:
	push {r4, r5, lr}
	adds r5, r0, #0	@ keep the command across the delay
	ldr r4, .Lsc_ctrl
	movs r3, #1
	strb r3, [r4]	@ 0x80000d = 1
	movs r0, #1
	bl tlsr_flash_delay_us	@ for 1 us
	movs r3, #0
	strb r3, [r4]	@ 0x80000d = 0
	ldr r3, .Lsc_data
	strb r5, [r3]	@ the command byte to 0x80000c
	movs r2, #0x10
1:	ldrb r3, [r4]
	tst r2, r3
	bne 1b	@ until 0x80000d bit 4 reads 0
	pop {r4, r5, pc}
	.short	0x46c0			@ pad (never executed)
.Lsc_ctrl:	.word	0x0080000d
.Lsc_data:	.word	0x0080000c
	.size tlsr_flash_send_cmd, . - tlsr_flash_send_cmd

/*
 * wait_ready(): sleep_us(100), send_cmd(0x05), then up to 10,000,000 times:
 * 0 to 0x80000c, the wait for 0x80000d bit 4 = 0, the status byte from
 * 0x80000c; stop when its bit 0 reads 0. Then 0x80000d = 1.
 */
	.type tlsr_flash_wait_ready, %function
tlsr_flash_wait_ready:
	push {r4, r5, r6, lr}
	movs r0, #100
	bl tlsr_flash_delay_us	@ 100 us before the first status read
	movs r0, #5
	bl tlsr_flash_send_cmd	@ read status register
	ldr r0, .Lwd_limit	@ the number of status reads
	ldr r4, .Lwd_data
	movs r6, #0
	ldr r1, .Lwd_ctrl
	movs r2, #0x10
	movs r5, #1	@ status bit 0
1:	strb r6, [r4]	@ 0 to 0x80000c
2:	ldrb r3, [r1]
	tst r2, r3
	bne 2b	@ until 0x80000d bit 4 reads 0
	ldrb r3, [r4]	@ the status byte
	tst r5, r3
	beq 3f	@ bit 0 = 0: done
	subs r0, #1
	cmp r0, #0
	bne 1b
3:	movs r2, #1
	ldr r3, .Lwd_ctrl
	strb r2, [r3]	@ 0x80000d = 1
	pop {r4, r5, r6, pc}
	.short	0x46c0			@ pad (never executed)
.Lwd_limit:	.word	10000000
.Lwd_data:	.word	0x0080000c
.Lwd_ctrl:	.word	0x0080000d
	.size tlsr_flash_wait_ready, . - tlsr_flash_wait_ready

/*
 * read_cmd(cmd in r0, addr in r1, with_addr in r2, zeros in r3; on the
 * stack: buf, len): with 0x800643 = 0, send_cmd(cmd); send_addr(addr) when
 * with_addr is not 0; zeros 0 bytes to 0x80000c; one more 0; 0x80000d =
 * 0x0a; then len bytes read from 0x80000c into buf; 0x80000d = 1; 0x800643
 * as it was. Each byte written or read is followed by the wait for 0x80000d
 * bit 4 = 0, and so is the write of 0x0a.
 */
	.type tlsr_flash_read_cmd, %function
tlsr_flash_read_cmd:
	push {r4, r5, r6, r7, lr}
	mov r7, r10
	mov r6, r8
	push {r6, r7}	@ r8 and r10 are used below: saved as the ABI requires
	mov r10, r1	@ the address, kept across the calls
	ldr r6, [sp, #0x1c]	@ buf (fifth argument)
	ldr r4, [sp, #0x20]	@ len (sixth argument)
	lsls r0, r0, #24	@ cmd as a byte
	lsrs r0, r0, #24
	lsls r2, r2, #24	@ with_addr as a byte ...
	lsrs r7, r2, #24	@ ... in r7
	lsls r5, r3, #24	@ zeros as a byte ...
	lsrs r5, r5, #24	@ ... in r5
	ldr r3, .Lrd_irq
	ldrb r2, [r3]
	mov r8, r2	@ 0x800643 as it was
	movs r2, #0
	strb r2, [r3]	@ 0x800643 = 0: interrupts off
	bl tlsr_flash_send_cmd	@ the command
	cmp r7, #0
	bne .Lrd_addr	@ with an address: send it first
.Lrd_zeros:
	cmp r5, #0
	beq .Lrd_clock	@ no zero bytes
	movs r0, #0
	ldr r2, .Lrd_data
	mov r12, r2
	movs r7, #0
	ldr r1, .Lrd_ctrl
	movs r2, #0x10
1:	mov r3, r12
	strb r7, [r3]	@ 0 to 0x80000c
2:	ldrb r3, [r1]
	tst r2, r3
	bne 2b	@ until 0x80000d bit 4 reads 0
	adds r0, #1
	cmp r0, r5
	blt 1b	@ zeros of them
.Lrd_clock:
	movs r2, #0
	ldr r3, .Lrd_data
	strb r2, [r3]	@ one more 0 to 0x80000c
	ldr r0, .Lrd_ctrl
	movs r1, #0x10
1:	ldr r2, .Lrd_ctrl
	ldrb r3, [r0]
	tst r1, r3
	bne 1b	@ until 0x80000d bit 4 reads 0
	movs r3, #0x0a
	strb r3, [r2]	@ 0x80000d = 0x0a
	movs r1, #0x10
1:	ldrb r3, [r2]
	tst r1, r3
	bne 1b	@ until 0x80000d bit 4 reads 0
	cmp r4, #0
	beq .Lrd_end	@ nothing to read
	movs r0, #0
	ldr r5, .Lrd_data
	ldr r1, .Lrd_ctrl
	movs r2, #0x10
1:	ldrb r3, [r5]	@ a byte from 0x80000c
	strb r3, [r6, r0]	@ buf[i]
2:	ldrb r3, [r1]
	tst r2, r3
	bne 2b	@ until 0x80000d bit 4 reads 0
	adds r0, #1
	cmp r0, r4
	bne 1b	@ len of them
.Lrd_end:
	movs r2, #1
	ldr r3, .Lrd_ctrl
	strb r2, [r3]	@ 0x80000d = 1
	ldr r3, .Lrd_irq
	mov r2, r8
	strb r2, [r3]	@ 0x800643 as it was
	pop {r2, r3}
	mov r8, r2
	mov r10, r3
	pop {r4, r5, r6, r7, pc}
.Lrd_addr:
	mov r0, r10
	bl tlsr_flash_send_addr
	b .Lrd_zeros
	.short	0x46c0			@ pad (never executed)
.Lrd_irq:	.word	0x00800643
.Lrd_data:	.word	0x0080000c
.Lrd_ctrl:	.word	0x0080000d
	.size tlsr_flash_read_cmd, . - tlsr_flash_read_cmd

/*
 * write_cmd(cmd in r0, addr in r1, with_addr in r2, buf in r3; on the
 * stack: len): with 0x800643 = 0, send_cmd(0x06), send_cmd(cmd);
 * send_addr(addr) when with_addr is not 0; len bytes from buf to 0x80000c,
 * each followed by the wait for 0x80000d bit 4 = 0; 0x80000d = 1;
 * wait_ready; 0x800643 as it was.
 */
	.type tlsr_flash_write_cmd, %function
tlsr_flash_write_cmd:
	push {r4, r5, r6, r7, lr}
	mov r7, r10
	mov r6, r8
	push {r6, r7}
	mov r10, r1	@ the address
	adds r5, r3, #0	@ buf
	ldr r4, [sp, #0x1c]	@ len (fifth argument)
	lsls r0, r0, #24
	lsrs r6, r0, #24	@ cmd as a byte, in r6
	lsls r2, r2, #24
	lsrs r7, r2, #24	@ with_addr as a byte, in r7
	ldr r3, .Lwr_irq
	ldrb r2, [r3]
	mov r8, r2	@ 0x800643 as it was
	movs r2, #0
	strb r2, [r3]	@ 0x800643 = 0: interrupts off
	movs r0, #6
	bl tlsr_flash_send_cmd	@ write enable
	adds r0, r6, #0
	bl tlsr_flash_send_cmd	@ the command
	cmp r7, #0
	bne .Lwr_addr
.Lwr_bytes:
	cmp r4, #0
	beq .Lwr_end	@ no data
	movs r3, #0
	movs r0, #0
	ldr r6, .Lwr_data
	ldr r1, .Lwr_ctrl
	movs r2, #0x10
1:	ldrb r3, [r5, r3]	@ buf[i]
	strb r3, [r6]	@ to 0x80000c
2:	ldrb r3, [r1]
	tst r2, r3
	bne 2b	@ until 0x80000d bit 4 reads 0
	adds r0, #1
	adds r3, r0, #0	@ the index for the next load
	cmp r0, r4
	bne 1b	@ len of them
.Lwr_end:
	movs r2, #1
	ldr r3, .Lwr_ctrl
	strb r2, [r3]	@ 0x80000d = 1
	bl tlsr_flash_wait_ready
	ldr r3, .Lwr_irq
	mov r2, r8
	strb r2, [r3]	@ 0x800643 as it was
	pop {r2, r3}
	mov r8, r2
	mov r10, r3
	pop {r4, r5, r6, r7, pc}
.Lwr_addr:
	mov r0, r10
	bl tlsr_flash_send_addr
	b .Lwr_bytes
.Lwr_irq:	.word	0x00800643
.Lwr_data:	.word	0x0080000c
.Lwr_ctrl:	.word	0x0080000d
	.size tlsr_flash_write_cmd, . - tlsr_flash_write_cmd

/*
 * sleep_us(us in r0): wait until more than us microseconds of the system timer
 * (16 ticks each) have passed since the call. The subtraction wraps, so the
 * timer passing 0xffffffff on the way does not matter.
 */
	.type tlsr_flash_delay_us, %function
tlsr_flash_delay_us:
	ldr r2, .Lsl_tick
	ldr r1, [r2]	@ the timer at the start
	lsls r0, r0, #4	@ us in timer ticks
1:	ldr r3, [r2]
	subs r3, r3, r1	@ ticks since the start
	cmp r0, r3
	bhs 1b	@ until more than us * 16 have passed
	bx lr
.Lsl_tick:	.word	0x00800740	@ system timer
	.size tlsr_flash_delay_us, . - tlsr_flash_delay_us

	.size tlsr_spi_flash_io, . - tlsr_spi_flash_io

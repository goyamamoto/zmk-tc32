/*
 * The SPI flash transport: sending a command and an address to the flash,
 * reading and writing through 0x80000c/0x80000d, waiting for a program or
 * erase to finish, and the microsecond delay they use. Its 496 bytes must
 * equal tlsr_spi_flash_io.json's blob_sha256, which every Thumb build
 * (THUMB=1) checks (tc32-devtools compiler/image_check.py --blob).
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
 * tnand rn, rm sets the flags from rn & rm and writes no register.
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
	tshftl	r2, r0, #8	@ bits 23:16 of the address ...
	tshftr	r2, r2, #24	@ ... alone in r2
	tloadr	r3, .Lsa_data
	tstorerb	r2, [r3]	@ to 0x80000c
	tloadr	r1, .Lsa_ctrl
	tmov	r2, #0x10	@ 0x80000d bit 4
1:	tloadrb	r3, [r1]
	tnand	r2, r3
	tjne	1b		@ until 0x80000d bit 4 reads 0
	tshftl	r2, r0, #16	@ bits 15:8
	tshftr	r2, r2, #24
	tloadr	r3, .Lsa_data
	tstorerb	r2, [r3]
	tloadr	r1, .Lsa_ctrl
	tmov	r2, #0x10
1:	tloadrb	r3, [r1]
	tnand	r2, r3
	tjne	1b
	tshftl	r0, r0, #24	@ bits 7:0
	tshftr	r0, r0, #24
	tloadr	r3, .Lsa_data
	tstorerb	r0, [r3]
	tloadr	r1, .Lsa_ctrl
	tmov	r2, #0x10
1:	tloadrb	r3, [r1]
	tnand	r2, r3
	tjne	1b
	tjex	lr
.Lsa_data:	.word	0x0080000c
.Lsa_ctrl:	.word	0x0080000d
	.size tlsr_flash_send_addr, . - tlsr_flash_send_addr

/*
 * send_cmd(cmd in r0): 0x80000d = 1, sleep_us(1), 0x80000d = 0, the command
 * byte to 0x80000c, the wait for 0x80000d bit 4 = 0.
 */
	.type tlsr_flash_send_cmd, %function
tlsr_flash_send_cmd:
	tpush	{r4, r5, lr}
	tmov	r5, r0		@ keep the command across the delay
	tloadr	r4, .Lsc_ctrl
	tmov	r3, #1
	tstorerb	r3, [r4]	@ 0x80000d = 1
	tmov	r0, #1
	tjl	tlsr_flash_delay_us	@ for 1 us
	tmov	r3, #0
	tstorerb	r3, [r4]	@ 0x80000d = 0
	tloadr	r3, .Lsc_data
	tstorerb	r5, [r3]	@ the command byte to 0x80000c
	tmov	r2, #0x10
1:	tloadrb	r3, [r4]
	tnand	r2, r3
	tjne	1b		@ until 0x80000d bit 4 reads 0
	tpop	{r4, r5, pc}
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
	tpush	{r4, r5, r6, lr}
	tmov	r0, #100
	tjl	tlsr_flash_delay_us	@ 100 us before the first status read
	tmov	r0, #5
	tjl	tlsr_flash_send_cmd	@ read status register
	tloadr	r0, .Lwd_limit	@ the number of status reads
	tloadr	r4, .Lwd_data
	tmov	r6, #0
	tloadr	r1, .Lwd_ctrl
	tmov	r2, #0x10
	tmov	r5, #1		@ status bit 0
1:	tstorerb	r6, [r4]	@ 0 to 0x80000c
2:	tloadrb	r3, [r1]
	tnand	r2, r3
	tjne	2b		@ until 0x80000d bit 4 reads 0
	tloadrb	r3, [r4]	@ the status byte
	tnand	r5, r3
	tjeq	3f		@ bit 0 = 0: done
	tsub	r0, #1
	tcmp	r0, #0
	tjne	1b
3:	tmov	r2, #1
	tloadr	r3, .Lwd_ctrl
	tstorerb	r2, [r3]	@ 0x80000d = 1
	tpop	{r4, r5, r6, pc}
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
	tpush	{r4, r5, r6, r7, lr}
	tmov	r7, r10
	tmov	r6, r8
	tpush	{r6, r7}	@ r8 and r10 are used below: saved as the ABI requires
	tmov	r10, r1		@ the address, kept across the calls
	tloadr	r6, [sp, #0x1c]	@ buf (fifth argument)
	tloadr	r4, [sp, #0x20]	@ len (sixth argument)
	tshftl	r0, r0, #24	@ cmd as a byte
	tshftr	r0, r0, #24
	tshftl	r2, r2, #24	@ with_addr as a byte ...
	tshftr	r7, r2, #24	@ ... in r7
	tshftl	r5, r3, #24	@ zeros as a byte ...
	tshftr	r5, r5, #24	@ ... in r5
	tloadr	r3, .Lrd_irq
	tloadrb	r2, [r3]
	tmov	r8, r2		@ 0x800643 as it was
	tmov	r2, #0
	tstorerb	r2, [r3]	@ 0x800643 = 0: interrupts off
	tjl	tlsr_flash_send_cmd	@ the command
	tcmp	r7, #0
	tjne	.Lrd_addr	@ with an address: send it first
.Lrd_zeros:
	tcmp	r5, #0
	tjeq	.Lrd_clock	@ no zero bytes
	tmov	r0, #0
	tloadr	r2, .Lrd_data
	tmov	r12, r2
	tmov	r7, #0
	tloadr	r1, .Lrd_ctrl
	tmov	r2, #0x10
1:	tmov	r3, r12
	tstorerb	r7, [r3]	@ 0 to 0x80000c
2:	tloadrb	r3, [r1]
	tnand	r2, r3
	tjne	2b		@ until 0x80000d bit 4 reads 0
	tadd	r0, #1
	tcmp	r0, r5
	tjlt	1b		@ zeros of them
.Lrd_clock:
	tmov	r2, #0
	tloadr	r3, .Lrd_data
	tstorerb	r2, [r3]	@ one more 0 to 0x80000c
	tloadr	r0, .Lrd_ctrl
	tmov	r1, #0x10
1:	tloadr	r2, .Lrd_ctrl
	tloadrb	r3, [r0]
	tnand	r1, r3
	tjne	1b		@ until 0x80000d bit 4 reads 0
	tmov	r3, #0x0a
	tstorerb	r3, [r2]	@ 0x80000d = 0x0a
	tmov	r1, #0x10
1:	tloadrb	r3, [r2]
	tnand	r1, r3
	tjne	1b		@ until 0x80000d bit 4 reads 0
	tcmp	r4, #0
	tjeq	.Lrd_end	@ nothing to read
	tmov	r0, #0
	tloadr	r5, .Lrd_data
	tloadr	r1, .Lrd_ctrl
	tmov	r2, #0x10
1:	tloadrb	r3, [r5]	@ a byte from 0x80000c
	tstorerb	r3, [r6, r0]	@ buf[i]
2:	tloadrb	r3, [r1]
	tnand	r2, r3
	tjne	2b		@ until 0x80000d bit 4 reads 0
	tadd	r0, #1
	tcmp	r0, r4
	tjne	1b		@ len of them
.Lrd_end:
	tmov	r2, #1
	tloadr	r3, .Lrd_ctrl
	tstorerb	r2, [r3]	@ 0x80000d = 1
	tloadr	r3, .Lrd_irq
	tmov	r2, r8
	tstorerb	r2, [r3]	@ 0x800643 as it was
	tpop	{r2, r3}
	tmov	r8, r2
	tmov	r10, r3
	tpop	{r4, r5, r6, r7, pc}
.Lrd_addr:
	tmov	r0, r10
	tjl	tlsr_flash_send_addr
	tj	.Lrd_zeros
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
	tpush	{r4, r5, r6, r7, lr}
	tmov	r7, r10
	tmov	r6, r8
	tpush	{r6, r7}
	tmov	r10, r1		@ the address
	tmov	r5, r3		@ buf
	tloadr	r4, [sp, #0x1c]	@ len (fifth argument)
	tshftl	r0, r0, #24
	tshftr	r6, r0, #24	@ cmd as a byte, in r6
	tshftl	r2, r2, #24
	tshftr	r7, r2, #24	@ with_addr as a byte, in r7
	tloadr	r3, .Lwr_irq
	tloadrb	r2, [r3]
	tmov	r8, r2		@ 0x800643 as it was
	tmov	r2, #0
	tstorerb	r2, [r3]	@ 0x800643 = 0: interrupts off
	tmov	r0, #6
	tjl	tlsr_flash_send_cmd	@ write enable
	tmov	r0, r6
	tjl	tlsr_flash_send_cmd	@ the command
	tcmp	r7, #0
	tjne	.Lwr_addr
.Lwr_bytes:
	tcmp	r4, #0
	tjeq	.Lwr_end	@ no data
	tmov	r3, #0
	tmov	r0, #0
	tloadr	r6, .Lwr_data
	tloadr	r1, .Lwr_ctrl
	tmov	r2, #0x10
1:	tloadrb	r3, [r5, r3]	@ buf[i]
	tstorerb	r3, [r6]	@ to 0x80000c
2:	tloadrb	r3, [r1]
	tnand	r2, r3
	tjne	2b		@ until 0x80000d bit 4 reads 0
	tadd	r0, #1
	tmov	r3, r0		@ the index for the next load
	tcmp	r0, r4
	tjne	1b		@ len of them
.Lwr_end:
	tmov	r2, #1
	tloadr	r3, .Lwr_ctrl
	tstorerb	r2, [r3]	@ 0x80000d = 1
	tjl	tlsr_flash_wait_ready
	tloadr	r3, .Lwr_irq
	tmov	r2, r8
	tstorerb	r2, [r3]	@ 0x800643 as it was
	tpop	{r2, r3}
	tmov	r8, r2
	tmov	r10, r3
	tpop	{r4, r5, r6, r7, pc}
.Lwr_addr:
	tmov	r0, r10
	tjl	tlsr_flash_send_addr
	tj	.Lwr_bytes
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
	tloadr	r2, .Lsl_tick
	tloadr	r1, [r2]	@ the timer at the start
	tshftl	r0, r0, #4	@ us in timer ticks
1:	tloadr	r3, [r2]
	tsub	r3, r3, r1	@ ticks since the start
	tcmp	r0, r3
	tjhs	1b		@ until more than us * 16 have passed
	tjex	lr
.Lsl_tick:	.word	0x00800740	@ system timer
	.size tlsr_flash_delay_us, . - tlsr_flash_delay_us

	.size tlsr_spi_flash_io, . - tlsr_spi_flash_io

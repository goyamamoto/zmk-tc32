/*
 * Register model of the Telink TLSR8278 USB device controller. DS-TLSR8278
 * does not document these registers; the model is the behavior the TLSR8278
 * USB driver relies on:
 *  - EP0: an 8-byte FIFO behind a data port (0x800101) that post-increments
 *    the pointer (0x800100); the host's packet sits in the FIFO before the
 *    SETUP/DATA interrupt, and the device's IN packet is what it wrote since it
 *    last reset the pointer. The control register (0x800102) arms a stage with
 *    EP0_CTRL_DATA_ACK/EP0_CTRL_DATA_STALL or STA_ACK/STA_STALL; the IRQ status (0x800103) is
 *    write-one-to-clear.
 *  - Data endpoints: slot i (= ep & 7) has a pointer, a data port into the
 *    256-byte USB RAM at its buffer address, and a control register whose busy
 *    bit is set by the device to hand over a packet and cleared by the
 *    hardware when the host takes it; USB_IRQ (0x800139) is write-one-to-clear.
 * Assumed beyond that: the EP0 pointer holds the byte count of a
 * received OUT packet, and the data interrupt fires when an IN packet is taken
 * (the host can take packets without raising it, to cover the driver's poll).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "tlsr_usb_model.h"

void posix_sw_set_pending_IRQ(unsigned int IRQn);

#define REG_BASE 0x800000U
#define REG_SIZE 0x10000U

#define A_EP0_PTR          0x800100U
#define A_EP0_DATA         0x800101U
#define A_EP0_CTRL         0x800102U
#define A_EP0_IRQ_STATUS   0x800103U
#define A_EP_PTR0          0x800110U
#define A_EP_DATA0         0x800118U
#define A_EP_CTRL0         0x800120U
#define A_EP_BUF0          0x800128U
#define A_EP_IRQ           0x800139U
#define A_EP_IRQ_MASK      0x80013aU
#define A_IRQ_SOURCE_B2    0x80064aU
#define A_ANALOG_PORT_ADDR 0x8000b8U
#define A_ANALOG_PORT_DATA 0x8000b9U
#define A_ANALOG_PORT_CTRL 0x8000baU

#define IRQ_EP0_SETUP  8
#define IRQ_EP0_DATA   9
#define IRQ_EP0_STATUS 10
#define IRQ_EP_DATA    12
#define IRQ_USB_RESET  17

#define EP0_IRQ_SETUP        BIT(4)
#define EP0_IRQ_DATA         BIT(5)
#define EP0_IRQ_STATUS_STAGE BIT(6)

#define EP0_CTRL_DATA_ACK     BIT(0)
#define EP0_CTRL_DATA_STALL   BIT(1)
#define EP0_CTRL_STATUS_ACK   BIT(2)
#define EP0_CTRL_STATUS_STALL BIT(3)
#define EP_CTRL_BUSY          BIT(0)
#define EP_CTRL_DATA0         BIT(2)
#define EP_CTRL_DATA1         BIT(3)

static uint8_t regs[REG_SIZE];
static uint8_t analog[256];
static uint8_t ep0_fifo[8];
static uint8_t ep0_ptr;
static bool ep0_overflow;
static uint8_t usb_ram[256];
static uint8_t ep_ptr[8];
static int ep0_ctrl; /* last value written to the EP0 control register, -1 if none */
static bool irq_lines = true;
static enum sim_ep0_out_ptr out_ptr_mode = SIM_PTR_COUNT;
static uint8_t toggle_next[8]; /* expected PID per endpoint slot: 0 = DATA0 */
static int toggle_checked;
static int toggle_errors;

static uint8_t *reg(uintptr_t addr)
{
	__ASSERT(addr >= REG_BASE && addr < REG_BASE + REG_SIZE, "address 0x%lx", (long)addr);
	return &regs[addr - REG_BASE];
}

uint8_t tlsr_usb_sim_rd8(uintptr_t addr)
{
	if (addr == A_EP0_PTR) {
		return ep0_ptr;
	}
	if (addr == A_EP0_DATA) {
		uint8_t v = ep0_fifo[ep0_ptr & 7U];

		ep0_ptr++;
		return v;
	}
	if (addr >= A_EP_PTR0 && addr < A_EP_PTR0 + 8U) {
		return ep_ptr[addr - A_EP_PTR0];
	}
	if (addr >= A_EP_DATA0 && addr < A_EP_DATA0 + 8U) {
		uint8_t slot = addr - A_EP_DATA0;
		uint8_t v = usb_ram[(uint8_t)(*reg(A_EP_BUF0 + slot) + ep_ptr[slot])];

		ep_ptr[slot]++;
		return v;
	}
	if (addr == A_ANALOG_PORT_CTRL) {
		return *reg(addr) & (uint8_t)~BIT(0); /* never busy */
	}
	return *reg(addr);
}

void tlsr_usb_sim_wr8(uintptr_t addr, uint8_t value)
{
	if (addr == A_EP0_PTR) {
		ep0_ptr = value;
		return;
	}
	if (addr == A_EP0_DATA) {
		if (ep0_ptr >= sizeof(ep0_fifo)) {
			ep0_overflow = true;
		}
		ep0_fifo[ep0_ptr & 7U] = value;
		ep0_ptr++;
		return;
	}
	if (addr == A_EP0_CTRL) {
		ep0_ctrl = value;
		return;
	}
	if (addr == A_EP0_IRQ_STATUS || addr == A_EP_IRQ || addr == A_IRQ_SOURCE_B2) {
		*reg(addr) &= (uint8_t)~value; /* write one to clear */
		return;
	}
	if (addr >= A_EP_PTR0 && addr < A_EP_PTR0 + 8U) {
		ep_ptr[addr - A_EP_PTR0] = value;
		return;
	}
	if (addr >= A_EP_DATA0 && addr < A_EP_DATA0 + 8U) {
		uint8_t slot = addr - A_EP_DATA0;

		usb_ram[(uint8_t)(*reg(A_EP_BUF0 + slot) + ep_ptr[slot])] = value;
		ep_ptr[slot]++;
		return;
	}
	if (addr == A_ANALOG_PORT_CTRL && (value & BIT(6)) != 0U) {
		if ((value & BIT(5)) != 0U) {
			analog[*reg(A_ANALOG_PORT_ADDR)] = *reg(A_ANALOG_PORT_DATA);
		} else {
			*reg(A_ANALOG_PORT_DATA) = analog[*reg(A_ANALOG_PORT_ADDR)];
		}
	}
	*reg(addr) = value;
}

/* ----------------------------------------------------------- host side */

void sim_set_irq_lines(bool on)
{
	irq_lines = on;
}

void sim_set_ep0_out_ptr(enum sim_ep0_out_ptr mode)
{
	out_ptr_mode = mode;
}

void sim_toggle_restart(void)
{
	memset(toggle_next, 0, sizeof(toggle_next));
}

void sim_toggle_stats(int *checked, int *errors)
{
	*checked = toggle_checked;
	*errors = toggle_errors;
}

/* Sets status bits and either pends the line or waits for the driver's poll to clear them. */
static void raise_bits(uintptr_t sta_reg, uint8_t bits, unsigned int irq)
{
	*reg(sta_reg) |= bits;
	if (irq_lines) {
		posix_sw_set_pending_IRQ(irq);
		k_yield();
		return;
	}
	for (int i = 0; i < 20 && (*reg(sta_reg) & bits) != 0U; i++) {
		k_msleep(1);
	}
}

static void raise(uint8_t sta, unsigned int irq)
{
	raise_bits(A_EP0_IRQ_STATUS, sta, irq);
}

bool sim_dp_pullup(void)
{
	return (analog[0x0b] & BIT(7)) != 0U;
}

void sim_bus_reset(void)
{
	sim_toggle_restart();
	raise_bits(A_IRQ_SOURCE_B2, BIT(1), IRQ_USB_RESET);
}

static int stage(uint8_t sta, unsigned int irq, uint8_t ack, uint8_t stall)
{
	ep0_ctrl = -1;
	raise(sta, irq);
	if (ep0_overflow) {
		return SIM_PROTOCOL;
	}
	if (ep0_ctrl >= 0 && (ep0_ctrl & stall) != 0) {
		return SIM_STALL;
	}
	if (ep0_ctrl < 0 || (ep0_ctrl & ack) == 0) {
		return SIM_NO_ACK;
	}
	return 0;
}

int sim_control(const uint8_t setup[8], const uint8_t *out, int out_len, uint8_t *in, int in_max)
{
	uint16_t wlength = setup[6] | (setup[7] << 8);
	int ret;

	ep0_overflow = false;
	memcpy(ep0_fifo, setup, 8);
	ep0_ptr = 8U;
	ret = stage(EP0_IRQ_SETUP, IRQ_EP0_SETUP, EP0_CTRL_DATA_ACK, EP0_CTRL_DATA_STALL);
	if (ret < 0) {
		return ret;
	}

	if ((setup[0] & 0x80U) != 0U) {
		int total = 0;

		for (;;) {
			/* IN token: the device's packet is what it wrote. */
			int n = ep0_ptr;

			if (n > 8) {
				return SIM_PROTOCOL;
			}
			if (in != NULL) {
				memcpy(&in[total], ep0_fifo, MIN(n, in_max - total));
			}
			total += n;
			ret = stage(EP0_IRQ_DATA, IRQ_EP0_DATA, EP0_CTRL_DATA_ACK, EP0_CTRL_DATA_STALL);
			if (ret < 0) {
				return ret;
			}
			if (n < 8 || total >= wlength) {
				break;
			}
		}
		/* Status stage: zero-length OUT. */
		ep0_ptr = 0U;
		ret = stage(EP0_IRQ_STATUS_STAGE, IRQ_EP0_STATUS, EP0_CTRL_STATUS_ACK, EP0_CTRL_STATUS_STALL);
		return ret < 0 ? ret : total;
	}

	for (int sent = 0; sent < out_len;) {
		int n = MIN(8, out_len - sent);

		memcpy(ep0_fifo, &out[sent], n);
		ep0_ptr = out_ptr_mode == SIM_PTR_COUNT ? n : out_ptr_mode == SIM_PTR_ZERO ? 0 : 8;
		ret = stage(EP0_IRQ_DATA, IRQ_EP0_DATA, EP0_CTRL_DATA_ACK, EP0_CTRL_DATA_STALL);
		if (ret < 0) {
			return ret;
		}
		sent += n;
	}
	/* Status stage: zero-length IN. */
	ret = stage(EP0_IRQ_STATUS_STAGE, IRQ_EP0_STATUS, EP0_CTRL_STATUS_ACK, EP0_CTRL_STATUS_STALL);
	return ret < 0 ? ret : 0;
}

int sim_control_abandon(const uint8_t setup[8], int in_packets)
{
	int total = 0;
	int ret;

	ep0_overflow = false;
	memcpy(ep0_fifo, setup, 8);
	ep0_ptr = 8U;
	ret = stage(EP0_IRQ_SETUP, IRQ_EP0_SETUP, EP0_CTRL_DATA_ACK, EP0_CTRL_DATA_STALL);
	if (ret < 0) {
		return ret;
	}
	for (int i = 0; i < in_packets; i++) {
		int n = ep0_ptr;

		if (n > 8) {
			return SIM_PROTOCOL;
		}
		total += n;
		ret = stage(EP0_IRQ_DATA, IRQ_EP0_DATA, EP0_CTRL_DATA_ACK, EP0_CTRL_DATA_STALL);
		if (ret < 0) {
			return ret;
		}
	}
	return total;
}

int sim_in_ep(uint8_t ep, uint8_t *buf, int max, bool raise_irq)
{
	uint8_t slot = ep & 7U;
	uint8_t ctrl = *reg(A_EP_CTRL0 + slot);
	int n;

	if ((ctrl & EP_CTRL_BUSY) == 0U) {
		return -1;
	}
	/* The PID the device armed: exactly one of DAT0/DAT1, alternating. */
	toggle_checked++;
	if ((ctrl & (EP_CTRL_DATA0 | EP_CTRL_DATA1)) !=
	    (toggle_next[slot] != 0U ? EP_CTRL_DATA1 : EP_CTRL_DATA0)) {
		toggle_errors++;
	}
	toggle_next[slot] ^= 1U;
	n = MIN(ep_ptr[slot], max);
	for (int i = 0; i < n; i++) {
		buf[i] = usb_ram[(uint8_t)(*reg(A_EP_BUF0 + slot) + i)];
	}
	*reg(A_EP_CTRL0 + slot) &= (uint8_t)~EP_CTRL_BUSY;
	if (raise_irq && irq_lines && (*reg(A_EP_IRQ_MASK) & BIT(slot)) != 0U) {
		*reg(A_EP_IRQ) |= BIT(slot);
		posix_sw_set_pending_IRQ(IRQ_EP_DATA);
		k_yield();
	}
	return n;
}

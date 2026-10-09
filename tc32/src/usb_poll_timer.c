/*
 * The USB controller's 1 ms poll from Timer1's interrupt
 * (USB_DC_TELINK_B87_EXTERNAL_POLL), for a board without the V21's LED key
 * matrix (src/led_key_matrix.c, which polls from its own Timer1 interrupt).
 *
 * The image polls the controller only (USB_DC_TELINK_B87_IRQ_LINES off), so
 * the poll runs every millisecond. From a kernel timer it costs the system
 * timer interrupt's timeout processing at every poll: the announce, the
 * timer's expiry and its next timeout, the next compare, mostly code that
 * misses the flash cache, about ten times the poll itself. Timer1 in mode 0
 * counts the system clock and starts again at its capture, so its interrupt
 * comes every millisecond with nothing to set; it calls the poll and nothing
 * else. The USB driver's own timer then polls every
 * USB_DC_TELINK_B87_FALLBACK_POLL_MS, and only when this one has not polled
 * since its last expiry: USB keeps working, slower, if Timer1 stops.
 *
 * With TLSR_USB_WIRED_POSITION_ONLY the timer starts when src/ble/ble.c
 * enables USB (usb_poll_timer_run()): at boot in the wired position, and in
 * the BT and 2.4G positions only once &usb_on asks for USB
 * (TLSR_USB_ON_REQUEST). Until then the low-power states of those links,
 * with the chip's idle suspend, have no 1 ms interrupt. Otherwise it starts
 * at init. It runs on through the host's USB suspend, as the V21's Timer1
 * does (src/usb_suspend.c; the poll does nothing while the driver has the
 * chip suspended).
 *
 * Timer1's registers: byte 0x620 (Timer1's enable and mode bits; 0x621-0x622
 * hold the watchdog's, 0x623 the statuses), its capture and its tick
 * (DS-TLSR8278 5.1.1). The boot guard's writes of 0x620-0x623 keep Timer1's
 * bits.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/drivers/usb/usb_dc_b87.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "usb_poll_timer.h"

/* reg_clk_sel 0x66 = 0x20, the SoC setup's: 48 MHz. */
#define SCLK_HZ       48000000U
#define PERIOD_CYCLES (SCLK_HZ / 1000U)

#define TMR_CTRL  0x00800620U
#define TMR1_EN   BIT(3)
#define TMR1_MODE (BIT(4) | BIT(5)) /* 0: the system clock */
#define TMR_STA   0x00800623U
#define TMR1_STA  BIT(1)
#define TMR1_CAPT 0x00800628U
#define TMR1_TICK 0x00800634U
#define IRQ_SRC   0x00800648U
#define IRQ_TMR1  1 /* reg_irq_mask bit 1; the arch clears the timer's status */

static bool running;

static void usb_poll_timer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	usb_dc_b87_poll();
}

/* Stopped, a match that came meanwhile cleared (the interrupt source, then the status). */
static void timer1_stop(void)
{
	sys_write8(sys_read8(TMR_CTRL) & (uint8_t)~(TMR1_EN | TMR1_MODE), TMR_CTRL);
	sys_write32(BIT(IRQ_TMR1), IRQ_SRC);
	sys_write8(TMR1_STA, TMR_STA);
}

void usb_poll_timer_run(bool run)
{
	unsigned int key = irq_lock();

	if (run != running) {
		running = run;
		timer1_stop();
		if (run) {
			sys_write32(0U, TMR1_TICK);
			sys_write32(PERIOD_CYCLES, TMR1_CAPT);
			sys_write8(sys_read8(TMR_CTRL) | TMR1_EN, TMR_CTRL);
		}
	}
	irq_unlock(key);
}

static int usb_poll_timer_init(void)
{
	IRQ_CONNECT(IRQ_TMR1, 0, usb_poll_timer_isr, NULL, 0);
	if (!IS_ENABLED(CONFIG_TLSR_USB_WIRED_POSITION_ONLY)) {
		usb_poll_timer_run(true);
	}
	irq_enable(IRQ_TMR1);
	return 0;
}

SYS_INIT(usb_poll_timer_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

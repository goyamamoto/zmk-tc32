/*
 * USB device controller driver (legacy usb_dc API) for the Telink TLSR8278.
 *
 * DS-TLSR8278 describes the USB block's endpoints (7.6), its interrupt
 * sources (6.2.1), the DP pull-up (afe_0x0b bit 7) and the USB wakeup (0x6e),
 * but not the controller's own registers (0x800100-0x80013f); for those the
 * comments give what the driver writes, reads and waits for. This driver has
 * been built and tested against a register model on native_sim, but not yet
 * run on hardware; the places that rest on assumptions are marked
 * "Unverified".
 *
 * Control endpoint (8-byte FIFO, serviced by software):
 *  - SETUP interrupt: read the 8 setup bytes; the stack answers inside the
 *    SETUP callback (first IN chunk, status ZLP, or stall); then arm the data
 *    stage with DAT_ACK (or DAT_STALL).
 *  - DATA interrupt: one data-stage packet has moved. IN: the stack writes the
 *    next chunk from the DATA_IN callback, then DAT_ACK. OUT: read the packet,
 *    hand it to the stack (DATA_OUT), then DAT_ACK.
 *  - STATUS interrupt: STA_ACK (or STA_STALL), then tell the stack.
 * The controller answers SET_ADDRESS itself (the driver never writes the
 * address); with bits 7, 5 and 1 of 0x104 cleared, standard, descriptor and
 * configuration requests go to software.
 *
 * Data endpoints: 1, 2, 3, 4, 7, 8 are IN, 5 and 6 are OUT. Register index is
 * (ep & 7), so endpoint 8 uses slot 0. Buffers come from the 256-byte USB RAM.
 *
 * Copyright (c) 2026 Go Yamamoto
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define DT_DRV_COMPAT telink_b87_usbd

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb/usb_dc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>
#if !defined(CONFIG_USB_DC_TELINK_B87_SIM)
#include <tlsr827x/irq.h> /* tlsr827x_irq_clear_parent() */
#endif
#if defined(CONFIG_SOC_TLSR8278_SUSPEND)
#include <tlsr8278_suspend.h>
#endif
#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
#include <zephyr/drivers/usb/usb_dc_b87.h>
#endif

LOG_MODULE_REGISTER(usb_dc_b87, CONFIG_USB_DRIVER_LOG_LEVEL);

#if defined(CONFIG_USB_DC_TELINK_B87_SIM)
/* Host tests: a register model provides these (see USB_DC_TELINK_B87_SIM). */
uint8_t tlsr_usb_sim_rd8(uintptr_t addr);
void tlsr_usb_sim_wr8(uintptr_t addr, uint8_t value);
#define RD8(addr)        tlsr_usb_sim_rd8(addr)
#define WR8(addr, value) tlsr_usb_sim_wr8((addr), (value))
#else
#define RD8(addr)        (*(volatile uint8_t *)(uintptr_t)(addr))
#define WR8(addr, value) (*(volatile uint8_t *)(uintptr_t)(addr) = (value))
#endif
#define SET8(addr, mask) WR8((addr), (uint8_t)(RD8(addr) | (mask)))
#define CLR8(addr, mask) WR8((addr), (uint8_t)(RD8(addr) & ~(mask)))

#define USB_BASE DT_INST_REG_ADDR(0)

/* Control endpoint */
#define EP0_PTR        (USB_BASE + 0x00)
#define EP0_DATA       (USB_BASE + 0x01)
#define EP0_CTRL       (USB_BASE + 0x02)
#define EP0_IRQ_STATUS (USB_BASE + 0x03)
#define EP0_IRQ_MODE   (USB_BASE + 0x04)

#define EP0_CTRL_DATA_ACK     BIT(0)
#define EP0_CTRL_DATA_STALL   BIT(1)
#define EP0_CTRL_STATUS_ACK   BIT(2)
#define EP0_CTRL_STATUS_STALL BIT(3)

#define EP0_IRQ_SETUP        BIT(4)
#define EP0_IRQ_DATA         BIT(5)
#define EP0_IRQ_STATUS_STAGE BIT(6)

#define EP0_AUTO_CFG  BIT(1)
#define EP0_AUTO_DESC BIT(5)
#define EP0_AUTO_STD  BIT(7)

/* Data endpoints; i is the register slot (ep & 7) */
#define EP_ENABLE      (USB_BASE + 0x0e)
#define EP_PTR(i)      (USB_BASE + 0x10 + (i))
#define EP_DATA(i)     (USB_BASE + 0x18 + (i))
#define EP_CTRL(i)     (USB_BASE + 0x20 + (i))
#define EP_BUF_ADDR(i) (USB_BASE + 0x28 + (i))
#define EP_IRQ         (USB_BASE + 0x39)
#define EP_IRQ_MASK    (USB_BASE + 0x3a)
#define EP_MAX_SIZE    (USB_BASE + 0x3e)

#define EP_CTRL_BUSY  BIT(0)
#define EP_CTRL_STALL BIT(1)
#define EP_CTRL_DATA0 BIT(2)
#define EP_CTRL_DATA1 BIT(3)

/* Outside the USB block */
#define CLKEN0                  0x00800063
#define CLKEN0_USB              BIT(3)
#define GPIO_PA_IE              0x00800581
#define GPIO_PA_GPIO            0x00800586
#define USB_PINS_PA             (BIT(5) | BIT(6)) /* PA5 = DM, PA6 = DP */
/* usb_reset (DS-TLSR8278 6.2.1); the poll reads it unmasked */
#define IRQ_SOURCE_B2           0x0080064a
#define IRQ_SOURCE_B2_USB_RESET BIT(1)
/* usb_pwdn (DS-TLSR8278 6.2.1): the host suspended the bus */
#define IRQ_SOURCE_B0           0x00800648
#define IRQ_SOURCE_B0_USB_PWDN  BIT(3)
#define USB_REG_0A              (USB_BASE + 0x0a)
#define USB_REG_0A_BIT2         BIT(2) /* not documented; see usb_dc_wakeup_request() */
#define WAKEUP_EN               0x0080006e /* wakeup enable (DS-TLSR8278 Table 2-7) */
#define WAKEUP_EN_USB           BIT(2)
#define WAKEUP_EN_USB_RESUME    BIT(6)

/* Analog registers */
#define ANALOG_PORT_ADDR  0x008000b8
#define ANALOG_PORT_DATA  0x008000b9
#define ANALOG_PORT_CTRL  0x008000ba
#define ANALOG_PORT_BUSY  BIT(0)
#define ANALOG_PORT_WRITE BIT(5)
#define ANALOG_PORT_START BIT(6)
#define ANA_USB_DP_PULLUP 0x0b /* bit 7: 1.5 kOhm pull-up on DP */
#define ANA_USB_POWER     0x34 /* bit 1 (not documented): cleared at attach */

#define EP0_MPS      8U
#define USB_RAM_SIZE 256U
#define NUM_EP_SLOTS 8U
#define EP_IN_MASK   (BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(7) | BIT(8))
#define EP_OUT_MASK  (BIT(5) | BIT(6))

struct tlsr_ep {
	usb_dc_ep_callback cb;
	uint16_t mps;
	bool configured;
	bool enabled;
	bool in_flight; /* IN: a packet is armed and not yet taken */
	bool toggle;    /* DATA1 next; the driver sets the PID */
};

static struct {
	usb_dc_status_callback status_cb;
	usb_dc_ep_callback ep0_out_cb;
	usb_dc_ep_callback ep0_in_cb;
	struct tlsr_ep ep[NUM_EP_SLOTS]; /* indexed by register slot */
	uint16_t ram_next;
	bool attached;
	bool suspended;

	/* Control transfer state */
	uint8_t setup[8];
	bool setup_valid; /* ep_read(0x00) returns the setup packet */
	bool dir_in;      /* data stage is device-to-host */
	bool stall;
	bool in_written;  /* the stack wrote this IN packet (possibly empty) */
	uint16_t out_remaining; /* OUT data stage bytes still expected (wLength) */
	uint8_t out_buf[EP0_MPS];
	uint8_t out_len;
	uint8_t out_pos;
} dev_data;

static struct k_timer poll_timer;

static uint8_t analog_read(uint8_t addr)
{
	unsigned int key = irq_lock();
	uint8_t data;

	WR8(ANALOG_PORT_ADDR, addr);
	WR8(ANALOG_PORT_CTRL, ANALOG_PORT_START);
	while ((RD8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	data = RD8(ANALOG_PORT_DATA);
	WR8(ANALOG_PORT_CTRL, 0U);
	irq_unlock(key);
	return data;
}

static void analog_write(uint8_t addr, uint8_t value)
{
	unsigned int key = irq_lock();

	WR8(ANALOG_PORT_ADDR, addr);
	WR8(ANALOG_PORT_DATA, value);
	WR8(ANALOG_PORT_CTRL, ANALOG_PORT_START | ANALOG_PORT_WRITE);
	while ((RD8(ANALOG_PORT_CTRL) & ANALOG_PORT_BUSY) != 0U) {
	}
	WR8(ANALOG_PORT_CTRL, 0U);
	irq_unlock(key);
}

static void dp_pullup(bool on)
{
	uint8_t v = analog_read(ANA_USB_DP_PULLUP);

	analog_write(ANA_USB_DP_PULLUP, on ? (v | BIT(7)) : (v & (uint8_t)~BIT(7)));
}

static inline uint8_t ep_num(uint8_t ep)
{
	return USB_EP_GET_IDX(ep);
}

static inline uint8_t ep_slot(uint8_t ep)
{
	return ep_num(ep) & 0x07U;
}

static inline uint8_t ep_bit(uint8_t ep)
{
	return BIT(ep_slot(ep)); /* EP8 is bit 0 in EP_EN, EP_IRQ and EP_IRQ_MASK */
}

static bool ep_valid(uint8_t ep)
{
	uint8_t n = ep_num(ep);

	if (n == 0U) {
		return true;
	}
	if (n > 8U) {
		return false;
	}
	return USB_EP_DIR_IS_IN(ep) ? (EP_IN_MASK & BIT(n)) != 0U : (EP_OUT_MASK & BIT(n)) != 0U;
}

static void notify(enum usb_dc_status_code code)
{
	if (dev_data.status_cb != NULL) {
		dev_data.status_cb(code, NULL);
	}
}

/* ------------------------------------------------------------------ EP0 */

static void ep0_setup_irq(void)
{
	WR8(EP0_PTR, 0U);
	for (int i = 0; i < 8; i++) {
		dev_data.setup[i] = RD8(EP0_DATA);
	}
	dev_data.setup_valid = true;
	dev_data.dir_in = (dev_data.setup[0] & USB_EP_DIR_MASK) != 0U;
	dev_data.stall = false;
	dev_data.in_written = false;
	dev_data.out_len = 0U;
	dev_data.out_pos = 0U;
	dev_data.out_remaining = dev_data.dir_in ? 0U : sys_get_le16(&dev_data.setup[6]);
	WR8(EP0_PTR, 0U);

	if (dev_data.ep0_out_cb != NULL) {
		dev_data.ep0_out_cb(USB_CONTROL_EP_OUT, USB_DC_EP_SETUP);
	}
	dev_data.setup_valid = false;

	WR8(EP0_CTRL, dev_data.stall ? EP0_CTRL_DATA_STALL : EP0_CTRL_DATA_ACK);
}

static void ep0_data_irq(void)
{
	if (dev_data.dir_in) {
		/* The previous IN packet was taken; the stack writes the next. */
		WR8(EP0_PTR, 0U);
		dev_data.in_written = false;
		if (dev_data.ep0_in_cb != NULL) {
			dev_data.ep0_in_cb(USB_CONTROL_EP_IN, USB_DC_EP_DATA_IN);
		}
	} else {
		/*
		 * The packet size follows from wLength: full packets of 8
		 * bytes, then the rest. The EP0 pointer is not known to hold
		 * the received count.
		 */
		uint8_t n = (uint8_t)MIN(dev_data.out_remaining, EP0_MPS);
		uint8_t ptr = RD8(EP0_PTR);

		if (ptr != n) {
			LOG_DBG("EP0 OUT: pointer %u, expected %u bytes", ptr, n);
		}
		dev_data.out_remaining -= n;
		WR8(EP0_PTR, 0U);
		for (uint8_t i = 0; i < n; i++) {
			dev_data.out_buf[i] = RD8(EP0_DATA);
		}
		dev_data.out_len = n;
		dev_data.out_pos = 0U;
		if (dev_data.ep0_out_cb != NULL) {
			dev_data.ep0_out_cb(USB_CONTROL_EP_OUT, USB_DC_EP_DATA_OUT);
		}
	}
	WR8(EP0_CTRL, dev_data.stall ? EP0_CTRL_DATA_STALL : EP0_CTRL_DATA_ACK);
}

static void ep0_status_irq(void)
{
	WR8(EP0_CTRL, dev_data.stall ? EP0_CTRL_STATUS_STALL : EP0_CTRL_STATUS_ACK);
	if (dev_data.stall) {
		return;
	}
	if (dev_data.dir_in) {
		/* Status stage is a zero-length OUT from the host. */
		dev_data.out_len = 0U;
		dev_data.out_pos = 0U;
		if (dev_data.ep0_out_cb != NULL) {
			dev_data.ep0_out_cb(USB_CONTROL_EP_OUT, USB_DC_EP_DATA_OUT);
		}
	} else if (dev_data.ep0_in_cb != NULL) {
		/* Status stage was our zero-length IN. */
		dev_data.ep0_in_cb(USB_CONTROL_EP_IN, USB_DC_EP_DATA_IN);
	}
}

/*
 * The interrupt controller's source flags for the USB lines: the datasheet
 * does not say whether they follow the controller's status bits or latch.
 * Each handler clears its source flag after the status bit, so a latched
 * flag cannot keep the line pending (the dispatcher would give up after
 * TC32_IRQ_MAX_DRAIN rounds), and a level flag simply reasserts while the
 * status bit is set.
 */
static void clear_parent(unsigned int irq)
{
#if !defined(CONFIG_USB_DC_TELINK_B87_SIM)
	tlsr827x_irq_clear_parent(irq);
#else
	ARG_UNUSED(irq);
#endif
}

static void ctrl_isr(const void *arg)
{
	ARG_UNUSED(arg);
	uint8_t sta = RD8(EP0_IRQ_STATUS);

	if ((sta & EP0_IRQ_SETUP) != 0U) {
		WR8(EP0_IRQ_STATUS, EP0_IRQ_SETUP);
		clear_parent(DT_INST_IRQ_BY_NAME(0, ep0_setup, irq));
		ep0_setup_irq();
	}
	if ((sta & EP0_IRQ_DATA) != 0U) {
		WR8(EP0_IRQ_STATUS, EP0_IRQ_DATA);
		clear_parent(DT_INST_IRQ_BY_NAME(0, ep0_data, irq));
		ep0_data_irq();
	}
	if ((sta & EP0_IRQ_STATUS_STAGE) != 0U) {
		WR8(EP0_IRQ_STATUS, EP0_IRQ_STATUS_STAGE);
		clear_parent(DT_INST_IRQ_BY_NAME(0, ep0_status, irq));
		ep0_status_irq();
	}
}

/* ------------------------------------------------------- data endpoints */

/* Arms a data endpoint with the next data PID. */
static void ep_arm(uint8_t slot)
{
	struct tlsr_ep *e = &dev_data.ep[slot];

	WR8(EP_CTRL(slot), EP_CTRL_BUSY | (e->toggle ? EP_CTRL_DATA1 : EP_CTRL_DATA0));
	e->toggle = !e->toggle;
}

static void in_done(uint8_t slot)
{
	struct tlsr_ep *e = &dev_data.ep[slot];
	uint8_t n = slot == 0U ? 8U : slot;

	if (e->in_flight && (RD8(EP_CTRL(slot)) & EP_CTRL_BUSY) == 0U) {
		e->in_flight = false;
		if (e->cb != NULL) {
			e->cb(USB_EP_DIR_IN | n, USB_DC_EP_DATA_IN);
		}
	}
}

static void ep_data_isr(const void *arg)
{
	ARG_UNUSED(arg);
	uint8_t pending = RD8(EP_IRQ);
	uint8_t irq = pending & RD8(EP_IRQ_MASK);

	/*
	 * Write one to clear: every pending bit, not only the masked ones, so
	 * the bit of a slot the driver does not handle cannot hold the line.
	 */
	WR8(EP_IRQ, pending);
	clear_parent(DT_INST_IRQ_BY_NAME(0, ep_data, irq));
	for (uint8_t slot = 0; slot < NUM_EP_SLOTS; slot++) {
		uint8_t n = slot == 0U ? 8U : slot;

		if ((irq & BIT(slot)) == 0U || !dev_data.ep[slot].enabled) {
			continue;
		}
		if ((EP_IN_MASK & BIT(n)) != 0U) {
			in_done(slot);
		} else if (dev_data.ep[slot].cb != NULL) {
			dev_data.ep[slot].cb(n, USB_DC_EP_DATA_OUT);
		}
	}
}

static void reset_isr(const void *arg)
{
	ARG_UNUSED(arg);
	if ((RD8(IRQ_SOURCE_B2) & IRQ_SOURCE_B2_USB_RESET) == 0U) {
		return;
	}
	WR8(IRQ_SOURCE_B2, IRQ_SOURCE_B2_USB_RESET);
	clear_parent(DT_INST_IRQ_BY_NAME(0, reset, irq));
	for (uint8_t slot = 0; slot < NUM_EP_SLOTS; slot++) {
		WR8(EP_CTRL(slot), 0U);
		dev_data.ep[slot].in_flight = false;
		dev_data.ep[slot].toggle = false;
	}
	dev_data.stall = false;
	notify(USB_DC_RESET);
}

/* The five interrupt lines, with every pending status bit and source flag cleared first. */
static void irq_lines(bool on)
{
	static const unsigned int lines[] = {
		DT_INST_IRQ_BY_NAME(0, ep0_setup, irq), DT_INST_IRQ_BY_NAME(0, ep0_data, irq),
		DT_INST_IRQ_BY_NAME(0, ep0_status, irq), DT_INST_IRQ_BY_NAME(0, ep_data, irq),
		DT_INST_IRQ_BY_NAME(0, reset, irq),
	};

	if (!IS_ENABLED(CONFIG_USB_DC_TELINK_B87_IRQ_LINES)) {
		return;
	}
	if (on) {
		WR8(EP0_IRQ_STATUS,
		    EP0_IRQ_SETUP | EP0_IRQ_DATA | EP0_IRQ_STATUS_STAGE);
		WR8(EP_IRQ, 0xffU);
		WR8(IRQ_SOURCE_B2, IRQ_SOURCE_B2_USB_RESET);
	}
	for (size_t i = 0; i < ARRAY_SIZE(lines); i++) {
		if (on) {
			clear_parent(lines[i]);
			irq_enable(lines[i]);
		} else {
			irq_disable(lines[i]);
		}
	}
}

/*
 * The driver takes the USB interrupts (CONFIG_USB_DC_TELINK_B87_IRQ_LINES;
 * without it, it polls only) and also polls every millisecond (its k_timer,
 * or the board's timer with CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL), so it
 * works if a line never fires. Every handler acts only on status bits it
 * then clears, so an event seen by both runs once. The poll also completes
 * IN packets whose data interrupt does not come; usb_dc_ep_write()
 * completes them sooner when it finds the busy bit clear.
 */
/*
 * Suspend: usb_pwdn (0x648 bit 3), a level-triggered source (DS-TLSR8278
 * 6.2.1), is set while the host holds the bus suspended. Polled here; its
 * interrupt is not enabled.
 */
static void suspend_check(void)
{
	bool suspended = (RD8(IRQ_SOURCE_B0) & IRQ_SOURCE_B0_USB_PWDN) != 0U;

	if (suspended != dev_data.suspended) {
		dev_data.suspended = suspended;
		notify(suspended ? USB_DC_SUSPEND : USB_DC_RESUME);
	}
}

static void usb_poll(struct k_timer *timer)
{
	unsigned int key = irq_lock();

	ARG_UNUSED(timer);
	if ((RD8(EP0_IRQ_STATUS) &
	     (EP0_IRQ_SETUP | EP0_IRQ_DATA | EP0_IRQ_STATUS_STAGE)) != 0U) {
		ctrl_isr(NULL);
	}
	reset_isr(NULL);
	suspend_check();
	if ((RD8(EP_IRQ) & RD8(EP_IRQ_MASK)) != 0U) {
		ep_data_isr(NULL);
	}
	for (uint8_t slot = 0; slot < NUM_EP_SLOTS; slot++) {
		in_done(slot);
	}
	irq_unlock(key);
}

#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
/*
 * The board polls every millisecond (usb_dc_b87_poll()); the poll timer
 * is a fallback that polls only when the board has not polled since its last
 * expiry, so that USB (and with it the host's tools) still works if the
 * board's timer interrupt stops.
 *
 * The flags are plain volatile bytes: a byte store is one instruction, and
 * the two interrupts that share board_polled (the board's timer and the
 * system timer) do not nest on the TC32. The atomic_t API costs a call into
 * flash under an interrupt lock on this port, several times a millisecond.
 */
#define POLL_PERIOD  K_MSEC(CONFIG_USB_DC_TELINK_B87_FALLBACK_POLL_MS)
#define POLL_HANDLER fallback_poll

static volatile bool board_polled;
/*
 * The board's poll acts: from just after USB_DC_CONNECTED at attach until
 * detach, and not while the chip is in suspend. Set in thread context, read
 * in interrupts.
 */
static volatile bool board_poll_on;

void usb_dc_b87_poll(void)
{
	if (board_poll_on) {
		board_polled = true;
		usb_poll(NULL);
	}
}

static void fallback_poll(struct k_timer *timer)
{
	if (!board_polled) {
		usb_poll(timer);
	}
	board_polled = false;
}
#else
#define POLL_PERIOD  K_MSEC(1)
#define POLL_HANDLER usb_poll
#endif

#if defined(CONFIG_USB_DC_TELINK_B87_SLOW_POLL)
/*
 * A board's low-power state, while no host is using the bus, polls every
 * USB_DC_TELINK_B87_SLOW_POLL_MS instead of every millisecond
 * (usb_dc_b87_slow_poll()): an idle suspend needs the next kernel
 * timeout a few milliseconds away, and the poll timer's would always be
 * the next. Set and read in thread context.
 */
static bool slow_poll;
/*
 * Set while usb_dc_b87_chip_suspend() has the poll stopped around the
 * chip's suspend: a change of period then is only recorded, and the wake
 * starts the timer with it.
 */
static bool poll_stopped;
#define CUR_PERIOD (slow_poll ? K_MSEC(CONFIG_USB_DC_TELINK_B87_SLOW_POLL_MS) : POLL_PERIOD)

void usb_dc_b87_slow_poll(bool slow)
{
	unsigned int key = irq_lock();

	if (slow != slow_poll) {
		slow_poll = slow;
		if (dev_data.attached && !poll_stopped) {
			k_timer_start(&poll_timer, CUR_PERIOD, CUR_PERIOD);
		}
	}
	irq_unlock(key);
}
#else
#define CUR_PERIOD POLL_PERIOD
#endif

#if defined(CONFIG_SOC_TLSR8278_SUSPEND)
/*
 * Around the chip's suspend (tlsr8278_suspend.h): the poll stops, and polls
 * once at the wake. A periodic timer would otherwise fire once for every
 * millisecond the chip slept, all at the wake.
 */
void usb_dc_b87_chip_suspend(bool entering)
{
	if (!dev_data.attached) {
		return;
	}
	if (entering) {
#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
		board_poll_on = false;
#endif
#if defined(CONFIG_USB_DC_TELINK_B87_SLOW_POLL)
		unsigned int key = irq_lock();

		poll_stopped = true;
		k_timer_stop(&poll_timer);
		irq_unlock(key);
#else
		k_timer_stop(&poll_timer);
#endif
	} else {
#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
		/*
		 * The first poll after the wake is the board's (within a
		 * millisecond) or the fallback's, not one from this thread: a
		 * SETUP handled here could reach the stack's k_usleep() or
		 * k_yield() inside usb_poll()'s interrupt lock, and the board's
		 * poll could then enter usb_poll() while this one is parked.
		 */
		board_polled = false;
		board_poll_on = true;
		k_timer_start(&poll_timer, POLL_PERIOD, POLL_PERIOD);
#else
		/*
		 * The first poll at once, from the timer's interrupt, not from
		 * this thread: a SETUP handled here could reach the stack's
		 * k_usleep() or k_yield() inside usb_poll()'s interrupt lock, and
		 * the timer's poll could then enter usb_poll() while this one is
		 * parked.
		 */
#if defined(CONFIG_USB_DC_TELINK_B87_SLOW_POLL)
		unsigned int key = irq_lock();

		k_timer_start(&poll_timer, K_NO_WAIT, CUR_PERIOD);
		poll_stopped = false;
		irq_unlock(key);
#else
		k_timer_start(&poll_timer, K_NO_WAIT, CUR_PERIOD);
#endif
#endif
	}
}
#endif

/* ------------------------------------------------------------------ API */

int usb_dc_attach(void)
{
	if (dev_data.attached) {
		return 0;
	}

	/*
	 * USB power (analog 0x34 bit 1, not documented), clock (0x63 bit 3)
	 * and pins (PA5, PA6: GPIO function off, input on).
	 */
	analog_write(ANA_USB_POWER, analog_read(ANA_USB_POWER) & (uint8_t)~BIT(1));
	SET8(CLKEN0, CLKEN0_USB);
	CLR8(GPIO_PA_GPIO, USB_PINS_PA);
	SET8(GPIO_PA_IE, USB_PINS_PA);

	/* Standard, descriptor and configuration requests to software. */
	CLR8(EP0_IRQ_MODE, EP0_AUTO_STD | EP0_AUTO_DESC | EP0_AUTO_CFG);
	WR8(EP_MAX_SIZE, 64U >> 3);
	WR8(EP_ENABLE, 0U);
	WR8(EP_IRQ_MASK, 0U);
	dev_data.ram_next = 0U;

	irq_lines(true);

	dp_pullup(true);
	dev_data.attached = true;
#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
	board_polled = false; /* the fallback's first expiry polls unless the board has */
#endif
	k_timer_start(&poll_timer, CUR_PERIOD, CUR_PERIOD);
	notify(USB_DC_CONNECTED);
#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
	board_poll_on = true;
#endif
	return 0;
}

int usb_dc_detach(void)
{
#if defined(CONFIG_USB_DC_TELINK_B87_EXTERNAL_POLL)
	board_poll_on = false;
#endif
	dp_pullup(false);
	irq_lines(false);
	k_timer_stop(&poll_timer);
	dev_data.attached = false;
	notify(USB_DC_DISCONNECTED);
	return 0;
}

int usb_dc_reset(void)
{
	for (uint8_t slot = 0; slot < NUM_EP_SLOTS; slot++) {
		WR8(EP_CTRL(slot), 0U);
		dev_data.ep[slot] = (struct tlsr_ep){0};
	}
	WR8(EP_ENABLE, 0U);
	WR8(EP_IRQ_MASK, 0U);
	dev_data.ram_next = 0U;
	return 0;
}

int usb_dc_set_address(const uint8_t addr)
{
	/* The controller answers SET_ADDRESS itself (see the top of this file). */
	ARG_UNUSED(addr);
	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	dev_data.status_cb = cb;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	if (!ep_valid(cfg->ep_addr)) {
		return -EINVAL;
	}
	if (ep_num(cfg->ep_addr) == 0U) {
		return cfg->ep_type == USB_DC_EP_CONTROL && cfg->ep_mps <= EP0_MPS ? 0 : -EINVAL;
	}
	if (cfg->ep_type == USB_DC_EP_CONTROL || cfg->ep_mps > 64U) {
		return -EINVAL;
	}
	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t slot = ep_slot(cfg->ep_addr);
	uint16_t size = ROUND_UP(cfg->ep_mps, 8U);

	if (usb_dc_ep_check_cap(cfg) != 0) {
		return -EINVAL;
	}
	if (ep_num(cfg->ep_addr) == 0U) {
		return 0;
	}
	if (!dev_data.ep[slot].configured) {
		if (dev_data.ram_next + size > USB_RAM_SIZE) {
			LOG_ERR("USB RAM exhausted for ep 0x%02x", cfg->ep_addr);
			return -ENOMEM;
		}
		WR8(EP_BUF_ADDR(slot), (uint8_t)dev_data.ram_next);
		dev_data.ram_next += size;
	}
	dev_data.ep[slot].mps = cfg->ep_mps;
	dev_data.ep[slot].configured = true;
	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	if (!ep_valid(ep)) {
		return -EINVAL;
	}
	if (ep_num(ep) == 0U) {
		dev_data.stall = true; /* applied by the pending stage */
		return 0;
	}
	WR8(EP_CTRL(ep_slot(ep)), EP_CTRL_STALL);
	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	if (!ep_valid(ep)) {
		return -EINVAL;
	}
	if (ep_num(ep) == 0U) {
		dev_data.stall = false;
		return 0;
	}
	/* CLEAR_FEATURE(ENDPOINT_HALT) resets the data toggle. */
	dev_data.ep[ep_slot(ep)].toggle = false;
	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_arm(ep_slot(ep));
	} else {
		WR8(EP_CTRL(ep_slot(ep)), 0U);
	}
	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	if (!ep_valid(ep) || stalled == NULL) {
		return -EINVAL;
	}
	*stalled = ep_num(ep) == 0U ? dev_data.stall
				    : (RD8(EP_CTRL(ep_slot(ep))) & EP_CTRL_STALL) != 0U;
	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_enable(const uint8_t ep)
{
	uint8_t slot = ep_slot(ep);

	if (!ep_valid(ep)) {
		return -EINVAL;
	}
	if (ep_num(ep) == 0U) {
		return 0;
	}
	dev_data.ep[slot].enabled = true;
	dev_data.ep[slot].toggle = false; /* SET_CONFIGURATION/SET_INTERFACE: DATA0 */
	SET8(EP_ENABLE, ep_bit(ep));
	SET8(EP_IRQ_MASK, ep_bit(ep));
	if (USB_EP_DIR_IS_OUT(ep)) {
		WR8(EP_PTR(slot), 0U);
		ep_arm(slot); /* accept the first packet */
	}
	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	uint8_t slot = ep_slot(ep);

	if (!ep_valid(ep)) {
		return -EINVAL;
	}
	if (ep_num(ep) == 0U) {
		return 0;
	}
	CLR8(EP_IRQ_MASK, ep_bit(ep));
	CLR8(EP_ENABLE, ep_bit(ep));
	WR8(EP_CTRL(slot), 0U);
	dev_data.ep[slot].enabled = false;
	dev_data.ep[slot].in_flight = false;
	return 0;
}

int usb_dc_ep_flush(const uint8_t ep)
{
	if (!ep_valid(ep)) {
		return -EINVAL;
	}
	WR8(ep_num(ep) == 0U ? EP0_PTR : EP_PTR(ep_slot(ep)), 0U);
	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	uint32_t n;

	if (!ep_valid(ep) || !USB_EP_DIR_IS_IN(ep) || (data_len > 0U && data == NULL)) {
		return -EINVAL;
	}

	if (ep_num(ep) == 0U) {
		/* Filled into the FIFO; the stage in progress arms it. */
		n = MIN(data_len, EP0_MPS);
		if (!dev_data.in_written) {
			WR8(EP0_PTR, 0U);
		}
		for (uint32_t i = 0; i < n; i++) {
			WR8(EP0_DATA, data[i]);
		}
		dev_data.in_written = true;
		if (ret_bytes != NULL) {
			*ret_bytes = n;
		}
		return 0;
	}

	uint8_t slot = ep_slot(ep);
	struct tlsr_ep *e = &dev_data.ep[slot];
	unsigned int key = irq_lock();

	/*
	 * The host may have taken the last packet before its data interrupt was
	 * handled, or with no interrupt line at all, before the next poll.
	 * Complete it here (busy bit clear), so the caller does not wait for
	 * the poll. If the completion callback writes the next packet itself,
	 * this write gets -EAGAIN.
	 */
	in_done(slot);
	if (!e->enabled || e->in_flight || (RD8(EP_CTRL(slot)) & EP_CTRL_BUSY) != 0U) {
		irq_unlock(key);
		return -EAGAIN;
	}
	n = MIN(data_len, e->mps);
	WR8(EP_PTR(slot), 0U);
	for (uint32_t i = 0; i < n; i++) {
		WR8(EP_DATA(slot), data[i]);
	}
	ep_arm(slot); /* hand the packet to the host */
	e->in_flight = true;
	irq_unlock(key);

	if (ret_bytes != NULL) {
		*ret_bytes = n;
	}
	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
	uint32_t n;

	if (!ep_valid(ep) || !USB_EP_DIR_IS_OUT(ep)) {
		return -EINVAL;
	}

	if (ep_num(ep) == 0U) {
		if (dev_data.setup_valid) {
			n = MIN(max_data_len, sizeof(dev_data.setup));
			if (data != NULL) {
				memcpy(data, dev_data.setup, n);
			}
		} else {
			n = MIN(max_data_len, (uint32_t)(dev_data.out_len - dev_data.out_pos));
			if (data != NULL) {
				memcpy(data, &dev_data.out_buf[dev_data.out_pos], n);
			}
			dev_data.out_pos += n;
		}
		if (read_bytes != NULL) {
			*read_bytes = n;
		}
		return 0;
	}

	uint8_t slot = ep_slot(ep);
	uint8_t avail = RD8(EP_PTR(slot)); /* bytes received */

	n = data == NULL ? avail : MIN(max_data_len, avail);
	WR8(EP_PTR(slot), 0U);
	for (uint32_t i = 0; i < n && data != NULL; i++) {
		data[i] = RD8(EP_DATA(slot));
	}
	if (read_bytes != NULL) {
		*read_bytes = n;
	}
	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	if (!ep_valid(ep) || !USB_EP_DIR_IS_OUT(ep)) {
		return -EINVAL;
	}
	if (ep_num(ep) != 0U) {
		WR8(EP_PTR(ep_slot(ep)), 0U);
		ep_arm(ep_slot(ep)); /* accept the next packet */
	}
	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	int ret = usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes);

	if (ret == 0 && data != NULL) {
		ret = usb_dc_ep_read_continue(ep);
	}
	return ret;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	if (!ep_valid(ep)) {
		return -EINVAL;
	}
	if (ep_num(ep) == 0U) {
		if (USB_EP_DIR_IS_IN(ep)) {
			dev_data.ep0_in_cb = cb;
		} else {
			dev_data.ep0_out_cb = cb;
		}
		return 0;
	}
	dev_data.ep[ep_slot(ep)].cb = cb;
	return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	if (!ep_valid(ep)) {
		return -EINVAL;
	}
	return ep_num(ep) == 0U ? EP0_MPS : dev_data.ep[ep_slot(ep)].mps;
}

/*
 * Remote wakeup: -EACCES unless 0x10a bit 2 (not documented) is set; then,
 * while usb_pwdn is set, 0x6e = bit 6 (the chip sends the USB resume
 * signal, DS-TLSR8278 Table 2-7), then bit 2 (USB wakeup enable).
 * SET_FEATURE(DEVICE_REMOTE_WAKEUP) does not reach the stack (0x104's bits
 * other than 7, 5 and 1 keep their reset values), so Zephyr's
 * usb_wakeup_request(), which checks the stack's own record of the request,
 * refuses; callers use this function directly. Unverified on hardware.
 */
int usb_dc_wakeup_request(void)
{
	unsigned int key;

	if ((RD8(USB_REG_0A) & USB_REG_0A_BIT2) == 0U) {
		return -EACCES;
	}
	key = irq_lock();
	if ((RD8(IRQ_SOURCE_B0) & IRQ_SOURCE_B0_USB_PWDN) != 0U) {
		WR8(WAKEUP_EN, WAKEUP_EN_USB_RESUME);
		WR8(WAKEUP_EN, WAKEUP_EN_USB);
	}
	irq_unlock(key);
	return 0;
}

static int usb_dc_b87_init(void)
{
	k_timer_init(&poll_timer, POLL_HANDLER, NULL);

	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ep0_setup, irq), 0, ctrl_isr, NULL, 0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ep0_data, irq), 0, ctrl_isr, NULL, 0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ep0_status, irq), 0, ctrl_isr, NULL, 0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, ep_data, irq), 0, ep_data_isr, NULL, 0);
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, reset, irq), 0, reset_isr, NULL, 0);
	return 0;
}

SYS_INIT(usb_dc_b87_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

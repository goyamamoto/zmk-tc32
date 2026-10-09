/*
 * Register model of the Telink TLSR8278 USB device controller and the USB host
 * operations that drive it (host tests on native_sim).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TLSR_USB_MODEL_H_
#define TLSR_USB_MODEL_H_

#include <stdbool.h>
#include <stdint.h>

#define SIM_STALL    (-1)
#define SIM_NO_ACK   (-2)
#define SIM_PROTOCOL (-3)

/** @brief Whether the device enabled its DP pull-up (attached). */
bool sim_dp_pullup(void);

/** @brief Signal a USB bus reset. */
void sim_bus_reset(void);

/**
 * @brief Run one control transfer.
 *
 * @param setup   The 8-byte setup packet.
 * @param out     Data stage for host-to-device requests, or NULL.
 * @param out_len Length of @p out.
 * @param in      Buffer for device-to-host data, or NULL.
 * @param in_max  Size of @p in.
 *
 * @retval >=0 Bytes received in the data stage (0 for host-to-device).
 * @retval SIM_STALL The device stalled a stage.
 * @retval SIM_NO_ACK The device armed a stage with neither ACK nor STALL.
 * @retval SIM_PROTOCOL The device wrote more than 8 bytes into the EP0 FIFO.
 */
int sim_control(const uint8_t setup[8], const uint8_t *out, int out_len, uint8_t *in, int in_max);

/**
 * @brief Take the packet armed on an IN endpoint, as an IN token would.
 *
 * @param ep        Endpoint number (1-8).
 * @param buf       Buffer for the packet.
 * @param max       Size of @p buf.
 * @param raise_irq Also raise the endpoint data interrupt.
 *
 * @retval >=0 Packet length.
 * @retval -1 Nothing armed (NAK).
 */
int sim_in_ep(uint8_t ep, uint8_t *buf, int max, bool raise_irq);

/**
 * @brief Whether events pend the USB interrupt lines (default) or only set
 * status bits, which the driver then has to find by polling.
 */
void sim_set_irq_lines(bool on);

/** EP0 pointer after an OUT data packet (the hardware behavior is not known). */
enum sim_ep0_out_ptr {
	SIM_PTR_COUNT, /* the received byte count (default) */
	SIM_PTR_ZERO,
	SIM_PTR_EIGHT,
};
void sim_set_ep0_out_ptr(enum sim_ep0_out_ptr mode);

/**
 * @brief Data toggle check of interrupt IN packets.
 *
 * Every packet sim_in_ep() takes must be armed with exactly one of DAT0/DAT1,
 * alternating per endpoint from DATA0; sim_bus_reset() and
 * sim_toggle_restart() (after SET_CONFIGURATION) start again from DATA0.
 */
void sim_toggle_restart(void);
void sim_toggle_stats(int *checked, int *errors);

/**
 * @brief A control read the host gives up on: SETUP, then @p in_packets data
 * packets, no status stage (the next SETUP starts a new transfer).
 *
 * @return Bytes received, or SIM_* on error.
 */
int sim_control_abandon(const uint8_t setup[8], int in_packets);

#endif /* TLSR_USB_MODEL_H_ */

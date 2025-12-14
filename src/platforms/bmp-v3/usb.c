/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2025 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/bos.h>
#include <libopencm3/usb/dwc/otg_fs.h>
#include <libopencm3/../../lib/usb/usb_private.h>

#include "platform.h"

#undef OTG_FS_FIFO

/*
 * Receive FIFO size in 32-bit words.
 * We reserve first 4*n + 6 u32's for SETUP packets, where n is the number of endpoints total (6).
 * Next, we reserve our max packet size (64) / 4 + 1 for general packet storage, and an additional
 * one slot for the completion notification. This gives (4 * 6) + (64 / 4) + 8 = 24 + 16 + 8 = 48
 */
#define RX_FIFO_SIZE 48U /* 192 bytes */
/* FIFO access is made through pointers derived by this */
#define OTG_FS_FIFO(x) MMIO32(USB_OTG_FS_BASE + (0x1000U * (x + 1U)))

#define OTG_DCFG_ERRATIM (1U << 15U)

static usbd_device *bmd_dwc2_init(void);
void bmd_dwc2_disconnect(usbd_device *device, bool disconnect);
static void bmd_dwc2_set_address(usbd_device *device, uint8_t address);
static uint16_t bmd_dwc2_read_packet(usbd_device *device, uint8_t endpoint_address, void *buffer, uint16_t length);
static uint16_t bmd_dwc2_write_packet(
	usbd_device *device, uint8_t endpoint_address, const void *buffer, uint16_t length);
static void bmd_dwc2_stall_set(usbd_device *device, uint8_t endpoint_address, uint8_t stall);
static uint8_t bmd_dwc2_stall_get(usbd_device *device, uint8_t endpoint_address);
static void bmd_dwc2_nak_set(usbd_device *device, uint8_t endpoint_address, uint8_t nak);
static void bmd_dwc2_setup(usbd_device *device, uint8_t endpoint_address, uint8_t type, uint16_t max_packet_length,
	void (*callback)(usbd_device *usbd_dev, uint8_t ep));
static void bmd_dwc2_endpoints_reset(usbd_device *device);
static void bmd_dwc2_flush_txfifo(const uint8_t endpoint);
static void bmd_dwc2_poll(usbd_device *device);

static usbd_device usbd_dev;

const struct _usbd_driver bmd_dwc2_usb_driver = {
	.init = bmd_dwc2_init,
	.set_address = bmd_dwc2_set_address,
	.ep_setup = bmd_dwc2_setup,
	.ep_reset = bmd_dwc2_endpoints_reset,
	.ep_stall_set = bmd_dwc2_stall_set,
	.ep_stall_get = bmd_dwc2_stall_get,
	.ep_nak_set = bmd_dwc2_nak_set,
	.ep_write_packet = bmd_dwc2_write_packet,
	.ep_read_packet = bmd_dwc2_read_packet,
	.poll = bmd_dwc2_poll,
	.disconnect = bmd_dwc2_disconnect,
	.base_address = USB_OTG_FS_BASE,
	.set_address_before_status = true,
	.rx_fifo_size = RX_FIFO_SIZE,
};

static usbd_device *bmd_dwc2_init(void)
{
	/* Make sure the peripheral is clocked and do a full core reset having made sure the bus is idle */
	rcc_periph_clock_enable(RCC_OTGFS);
	while ((OTG_FS_GRSTCTL & OTG_GRSTCTL_AHBIDL) == 0U)
		continue;
	OTG_FS_GRSTCTL = OTG_GRSTCTL_CSRST;
	while ((OTG_FS_GRSTCTL & OTG_GRSTCTL_CSRST) != 0U)
		continue;

	/* Set the TX FIFO interrupt to work on empty, and disable global interrupts for the moment */
	OTG_FS_GAHBCFG = OTG_GAHBCFG_TXFELVL;
	/* Enable VBUS sensing in device mode, and power up the FS PHY */
	OTG_FS_GCCFG &= ~(OTG_GCCFG_PDEN | OTG_GCCFG_SDEN | OTG_GCCFG_DCDEN | OTG_GCCFG_BCDEN);
	OTG_FS_GCCFG |= OTG_GCCFG_VBDEN | OTG_GCCFG_PWRDWN;
	/* Set up for USB operation on a 160MHz AHB, don't enable HNP, or SRP and force device mode */
	OTG_FS_GUSBCFG = OTG_GUSBCFG_FDMOD | (6U << 10U);
	/* Clear all outstanding interrupts so we're in a clean state */
	OTG_FS_GINTSTS = UINT32_MAX;
	/*
	 * Unmask interrupts for core events - SOF, RX FIFO non-empty, USB suspend, USB reset,
	 * enumeration done, IN endpoint interrupts, amd wake-up detected
	 */
	OTG_FS_GINTMSK = OTG_GINTMSK_SOFM | OTG_GINTMSK_RXFLVLM | OTG_GINTMSK_USBSUSPM | OTG_GINTMSK_USBRST |
		OTG_GINTMSK_ENUMDNEM | OTG_GINTMSK_IEPINT | OTG_GINTMSK_OEPINT | OTG_GINTMSK_WUIM;

	/* Set up to operate as a USB FS device, resetting any other bits including device address */
	OTG_FS_DCFG &= ~(OTG_DCFG_ERRATIM | OTG_DCFG_PFIVL | OTG_DCFG_DAD | OTG_DCFG_NZLSOHSK);
	OTG_FS_DCFG |= OTG_DCFG_DSPD;

	/* Set up endpoint interrupts */
	OTG_FS_DAINTMSK = 0x003f003fU;
	/* Interrupt when IN transfer has completed */
	OTG_FS_DIEPMSK = OTG_DIEPMSK_XFRCM;
	/* Interrupt when OUT transfer has completed SETUP phase or a transfer complets */
	OTG_FS_DOEPMSK = OTG_DOEPMSK_STUPM | OTG_DOEPMSK_XFRCM;

	/* Enable global interrupts now we're all set */
	OTG_FS_GAHBCFG |= OTG_GAHBCFG_GINT;
	/* Ask the core to connect to USB */
	OTG_FS_DCTL &= ~OTG_DCTL_SDIS;
	return &usbd_dev;
}

void bmd_dwc2_disconnect(usbd_device *const device, const bool disconnect)
{
	(void)device;
	if (disconnect)
		OTG_FS_DCTL |= OTG_DCTL_SDIS;
	else
		OTG_FS_DCTL &= ~OTG_DCTL_SDIS;
}

static void bmd_dwc2_set_address(usbd_device *const device, const uint8_t address)
{
	(void)device;
	OTG_FS_DCFG = (OTG_FS_DCFG & ~OTG_DCFG_DAD) | ((address << 4U) & OTG_DCFG_DAD);
}

static uint16_t bmd_dwc2_read_packet(
	usbd_device *const device, const uint8_t endpoint_address, void *const buffer, const uint16_t length)
{
	/* We do not need to know the endpoint address since there is only one receive FIFO for all endpoints. */
	(void)endpoint_address;
	/* Figure out how many bytes to read, and how many can be read as u32 chunks */
	const size_t count = MIN(length, device->rxbcnt);
	const size_t aligned_count = count & ~3U;

	/* Copy the data out of the FIFO for this endpoint in u32 blocks */
	for (size_t offset = 0U; offset < aligned_count; offset += 4U)
		((uint32_t *)buffer)[offset >> 2U] = OTG_FS_FIFO(0U);

	/* If theres some data left over at the end, do the final copy */
	if (count - aligned_count) {
		/* Extract the last data block from the FIFO */
		const uint32_t data = OTG_FS_FIFO(0U);
		/* Copy the data for this final transfer into the target location in the buffer */
		memcpy((uint8_t *)buffer + aligned_count, &data, count - aligned_count);
		/* Because of how unloading works, we unload a bit more than this would ideally want */
		if (device->rxbcnt <= aligned_count + 4U)
			device->rxbcnt = 0U; /* If we exhausted the data, set to 0 */
		else
			device->rxbcnt -= count + 4U;
	} else
		/* All's said and done, so drop the read count by the amount read and return */
		device->rxbcnt -= count;
	return count;
}

static uint16_t bmd_dwc2_write_packet(
	usbd_device *const device, const uint8_t endpoint_address, const void *const buffer, const uint16_t length)
{
	(void)device;
	const uint8_t ep = endpoint_address & 0x7fU;
	/* Spin if endpoint is already enabled. */
	while ((OTG_FS_DIEPCTL(ep) & (OTG_DIEPCTL0_EPENA | OTG_DIEPCTL0_NAKSTS)) == OTG_DIEPCTL0_EPENA)
		continue;
	/* If it's still enabled but being NAK'd, flush FIFO and reset */
	if ((OTG_FS_DIEPCTL(ep) & OTG_DIEPCTL0_EPENA) != 0U) {
		bmd_dwc2_flush_txfifo(ep);
		/* Disable the endpoint and wait for it to become actually disabled */
		OTG_FS_DIEPCTL(ep) |= OTG_DIEPCTL0_EPDIS;
		while ((OTG_FS_DIEPINT(ep) & OTG_DIEPINTX_EPDISD) == 0U)
			continue;
		OTG_FS_DIEPINT(ep) = OTG_DIEPINTX_EPDISD;
	}

	/* Configure the endpoint to accept the new packet */
	if (ep == 0U)
		OTG_FS_DIEPTSIZ0 = OTG_DIEPSIZ0_PKTCNT | (length & OTG_DIEPSIZ0_XFRSIZ_MASK);
	else
		OTG_FS_DIEPTSIZ(ep) = OTG_DIEPSIZX_MCNT_1 | OTG_DIEPSIZX_PKTCNT(1) | (length & OTG_DIEPSIZX_XFRSIZ_MASK);
	/* Arm the endpoint for send */
	OTG_FS_DIEPCTL(ep) |= OTG_DIEPCTL0_EPENA | OTG_DIEPCTL0_CNAK;

	/* Figure out how many bytes can be written as u32 chunks */
	const size_t aligned_length = length & ~3U;
	/* Copy what we can into the FIFO for this endpoint in u32 blocks */
	for (size_t offset = 0U; offset < aligned_length; offset += 4U)
		OTG_FS_FIFO(ep) = ((const uint32_t *)buffer)[offset >> 2U];
	/* If there's some data left over at the end, do the final copy */
	if (length - aligned_length) {
		/* Prepare the data block for the FIFO */
		uint32_t data = 0U;
		memcpy(&data, (const uint8_t *)buffer + aligned_length, length - aligned_length);
		/* Push the prepared data into the FIFO to complete transfer setup */
		OTG_FS_FIFO(ep) = data;
	}
	/* Return that we wrote the whole packet out */
	return length;
}

static void bmd_dwc2_stall_set(usbd_device *const device, const uint8_t endpoint_address, const uint8_t stall)
{
	(void)device;
	/* Decode which endpoint this request is for exactly */
	const uint8_t ep = endpoint_address & 0x7fU;
	const uint8_t dir = endpoint_address & 0x80U;
	/* If the stall is for EP0, special-case to handle this correctly */
	if (ep == 0U) {
		/* Set/clear STALL on the IN side to properly communicate the condition back to the host */
		if (stall)
			OTG_FS_DIEPCTL(0U) |= OTG_DIEPCTL0_STALL;
		else
			OTG_FS_DIEPCTL(0U) &= ~OTG_DIEPCTL0_STALL;
	} else {
		/* Figure out which direction to set STALL for */
		if (dir == 0U) {
			/* Set/clear STALL on OUT endpoint */
			if (stall)
				OTG_FS_DOEPCTL(ep) |= OTG_DOEPCTL0_STALL;
			else
				OTG_FS_DOEPCTL(ep) &= ~OTG_DOEPCTL0_STALL;
			/* Reset DATA PID to use */
			OTG_FS_DOEPCTL(ep) |= OTG_DOEPCTLX_SD0PID;
		} else {
			/* Set/clear STALL on IN endpoint */
			if (stall)
				OTG_FS_DIEPCTL(ep) |= OTG_DIEPCTL0_STALL;
			else
				OTG_FS_DIEPCTL(ep) &= ~OTG_DIEPCTL0_STALL;
			/* Reset DATA PID to use */
			OTG_FS_DIEPCTL(ep) |= OTG_DIEPCTLX_SD0PID;
		}
	}
}

static uint8_t bmd_dwc2_stall_get(usbd_device *const device, const uint8_t endpoint_address)
{
	(void)device;
	/* Decode which endpoint this request is for exactly */
	const uint8_t ep = endpoint_address & 0x7fU;
	const uint8_t dir = endpoint_address & 0x80U;
	/* Handle OUT endpoints */
	if (dir == 0U)
		return (OTG_FS_DOEPCTL(ep) & OTG_DOEPCTL0_STALL) ? true : false;
	/* Handle IN endpoints */
	return (OTG_FS_DIEPCTL(ep) & OTG_DIEPCTL0_STALL) ? true : false;
}

static void bmd_dwc2_nak_set(usbd_device *const device, const uint8_t endpoint_address, const uint8_t nak)
{
	/* Decode which endpoint this request is for exactly */
	const uint8_t ep = endpoint_address & 0x7fU;
	const uint8_t dir = endpoint_address & 0x80U;
	/* Handle NAK's only on OUT endpoints */
	if (dir != 0U)
		return;
	/*
	 * Copy the required NAK state into the device state storage and then set
	 * the NAK bit for this endpoint accordingly via SNAK/CNAK
	 */
	device->force_nak[ep] = nak;
	if (nak)
		OTG_FS_DOEPCTL(ep) |= OTG_DOEPCTL0_SNAK;
	else
		OTG_FS_DOEPCTL(ep) |= OTG_DOEPCTL0_CNAK;
}

static void bmd_dwc2_setup(usbd_device *const device, const uint8_t endpoint_address, const uint8_t type,
	const uint16_t max_packet_length, void (*const callback)(usbd_device *usbd_dev, uint8_t ep))
{
	const uint8_t ep = endpoint_address & 0x7fU;
	const uint8_t dir = endpoint_address & 0x80U;
	/* Convert the max packet length to a length in u32's */
	const uint16_t packet_length = max_packet_length / 4U;

	/* Process if we're being asked to set up EP0, */
	if (ep == 0U) {
		/* Start by setting up the TX and RX FIFOs */
		OTG_FS_GRXFSIZ = device->driver->rx_fifo_size;
		OTG_FS_GNPTXFSIZ = (packet_length << 16U) | device->driver->rx_fifo_size;
		/* Update our internal state for how the FIFOs are presently allocated */
		device->fifo_mem_top_ep0 = device->driver->rx_fifo_size + packet_length;
		device->fifo_mem_top = device->fifo_mem_top_ep0;

		/* Configure EP0 IN to allow us to send packets appropriately */
		if (max_packet_length >= 64U)
			OTG_FS_DIEPCTL0 = OTG_DIEPCTL0_MPSIZ_64;
		else if (max_packet_length >= 32U)
			OTG_FS_DIEPCTL0 = OTG_DIEPCTL0_MPSIZ_32;
		else if (max_packet_length >= 16U)
			OTG_FS_DIEPCTL0 = OTG_DIEPCTL0_MPSIZ_16;
		else
			OTG_FS_DIEPCTL0 = OTG_DIEPCTL0_MPSIZ_8;

		/* Now configure EP0 OUT to allow us to receive SETUP packets */
		device->doeptsiz[0U] =
			OTG_DOEPSIZ0_STUPCNT_1 | OTG_DOEPSIZ0_PKTCNT | (max_packet_length & OTG_DOEPSIZ0_XFRSIZ_MASK);
		OTG_FS_DOEPTSIZ0 = device->doeptsiz[0U];
		/* Arm the endpoint to recieve packets */
		OTG_FS_DOEPCTL0 = OTG_DOEPCTL0_EPENA | OTG_DOEPCTL0_SNAK;
	} else {
		/* Otherwise process if this is for IN vs OUT */
		if (dir == 0U) {
			/* Set up this OUT endpoint, arming it so we can get data from it */
			device->doeptsiz[ep] = OTG_DOEPSIZX_PKTCNT(1U) | (max_packet_length & OTG_DOEPSIZX_XFRSIZ_MASK);
			OTG_FS_DOEPTSIZ(ep) = device->doeptsiz[ep];
			OTG_FS_DOEPCTL(ep) = OTG_DOEPCTL0_EPENA | OTG_DOEPCTL0_CNAK | OTG_DOEPCTL0_USBAEP | OTG_DOEPCTLX_SD0PID |
				(type << OTG_DOEPCTLX_EPTYP_SHIFT) | (max_packet_length & OTG_DOEPCTLX_MPSIZ_MASK);

			/* Install the user's callback if provided */
			device->user_callback_ctr[ep][USB_TRANSACTION_OUT] = callback;
		} else {
			/* Set up this IN endpoint, allocating space for it in the FIFO memory */
			OTG_FS_DIEPTXF(ep) = (packet_length << 16U) | device->fifo_mem_top;
			device->fifo_mem_top += packet_length;
			OTG_FS_DIEPTSIZ(ep) = 0U;
			/* Enable the endpoint but do not yet arm it as we've not yet got anything to send */
			OTG_FS_DIEPCTL(ep) = OTG_DIEPCTL0_SNAK | OTG_DIEPCTL0_USBAEP | OTG_DIEPCTLX_SD0PID |
				(ep << OTG_DIEPCTLX_TXFNUM_SHIFT) | (type << OTG_DIEPCTLX_EPTYP_SHIFT) |
				(max_packet_length & OTG_DIEPCTLX_MPSIZ_MASK);

			/* Install the user's callback if provided */
			device->user_callback_ctr[ep][USB_TRANSACTION_IN] = callback;
		}
	}
}

static void bmd_dwc2_endpoints_reset(usbd_device *const device)
{
	/* Start by resetting our FIFO setup state */
	device->fifo_mem_top = device->fifo_mem_top_ep0;

	/*
	 * Now loop through all endpoints and make sure we're NAK'ing and they're properly disabled
	 *
	 * NB: We ignore EP0 here because that's handled by the EP setup call _usbd_reset() performs.
	 */
	for (size_t i = 1U; i < ENDPOINT_COUNT; ++i) {
		OTG_FS_DOEPCTL(i) = OTG_DOEPCTL0_SNAK;
		if (OTG_FS_DOEPCTL(i) & OTG_DOEPCTL0_EPENA)
			OTG_FS_DOEPCTL(i) |= OTG_DOEPCTL0_EPDIS;
		if (OTG_FS_DIEPCTL(i) & OTG_DIEPCTL0_EPENA)
			OTG_FS_DIEPCTL(i) |= OTG_DIEPCTL0_EPDIS;
	}

	/* Make sure all FIFOs are fully flushed */
	OTG_FS_GRSTCTL = OTG_GRSTCTL_TXFNUM_ALL | OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_RXFFLSH;
	/* Wait for that to complete */
	while ((OTG_FS_GRSTCTL & (OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_RXFFLSH)) != 0U)
		continue;
	/* Reset the GRSTCTL register state */
	OTG_FS_GRSTCTL &= ~OTG_GRSTCTL_TXFNUM_MASK;
}

static void bmd_dwc2_flush_txfifo(const uint8_t endpoint)
{
	/* Mark the endpoint to NAK */
	OTG_FS_DIEPCTL(endpoint) |= OTG_DIEPCTL0_SNAK;
	while ((OTG_FS_DIEPINT(endpoint) & OTG_DIEPINTX_INEPNE) == 0)
		continue;
	/* Flush the FIFO requested */
	OTG_FS_GRSTCTL = (endpoint << 6U) | OTG_GRSTCTL_TXFFLSH;
	/* Wait for that to complete */
	while ((OTG_FS_GRSTCTL & OTG_GRSTCTL_TXFFLSH) != 0U)
		continue;
}

static void bmd_dwc2_poll(usbd_device *const device)
{
	const uint32_t status = OTG_FS_GINTSTS;
	/* First check to see if we're here for a USB reset event */
	if (status & OTG_GINTSTS_USBRST) {
		/* Do an endpoint reset, make sure EP0 is set up, and clear the condition */
		bmd_dwc2_endpoints_reset(device);
		_usbd_reset(device);
		OTG_FS_GINTSTS = OTG_GINTSTS_USBRST;
		/* Exit early as we're done here */
		return;
	}

	/* Now check to see if we're here for an enumeration done event */
	if (status & OTG_GINTSTS_ENUMDNE) {
		/* There's nothing much to do here, this interrupt just indicates that the link speed is now set */
		OTG_FS_GINTSTS = OTG_GINTSTS_ENUMDNE;
		return;
	}

	/* Handle notifications for IN transactions complete */
	if (status & OTG_GINTSTS_IEPINT) {
		for (size_t ep = 0U; ep < ENDPOINT_COUNT; ++ep) {
			/* If this endpoint has a completion, process it */
			if (OTG_FS_DIEPINT(ep) & OTG_DIEPINTX_XFRC) {
				/* Call any callback that might be available */
				if (device->user_callback_ctr[ep][USB_TRANSACTION_IN])
					device->user_callback_ctr[ep][USB_TRANSACTION_IN](device, ep);
				/* Clear the interrupt notification */
				OTG_FS_DIEPINT(ep) = OTG_DIEPINTX_XFRC;
			}
		}
	}

	/* Handle OUT packet reception */
	while (OTG_FS_GINTSTS & OTG_GINTSTS_RXFLVL) {
		/* Pop the RX packet status from the stack and decode */
		const uint32_t rx_status = OTG_FS_GRXSTSP;
		const uint32_t phase = rx_status & OTG_GRXSTSP_PKTSTS_MASK;
		const uint8_t ep = rx_status & OTG_GRXSTSP_EPNUM_MASK;
		device->rxbcnt = (rx_status & OTG_GRXSTSP_BCNT_MASK) >> 4U;

		switch (phase) {
		case OTG_GRXSTSP_PKTSTS_SETUP_COMP:
			/* Packet is for completion of a SETUP transaction, call the callback for this */
			device->user_callback_ctr[ep][USB_TRANSACTION_SETUP](device, ep);
			/* Mark it handled for this endpoint */
			OTG_FS_DOEPINT(ep) = OTG_DOEPINTX_STUP;
			break;
		case OTG_GRXSTSP_PKTSTS_SETUP:
			/* Packet is a SETUP packet, check if there's anything stuck in the TX FIFO to flush */
			if ((OTG_FS_DIEPTSIZ(ep) & OTG_DIEPSIZ0_PKTCNT) != 0U)
				bmd_dwc2_flush_txfifo(ep);
			/* Having made sure we're in a sensible state, now dequeue the data */
			bmd_dwc2_read_packet(device, ep, &device->control_state.req, sizeof(device->control_state.req));
			break;
		case OTG_GRXSTSP_PKTSTS_OUT:
			/* Call the user's handler if present */
			if (device->user_callback_ctr[ep][USB_TRANSACTION_OUT])
				device->user_callback_ctr[ep][USB_TRANSACTION_OUT](device, ep);
			break;
		default:
			break;
		}

		/* Discard any straggling data for this packet that wasn't yet handled */
		for (size_t offset = 0; offset < device->rxbcnt; offset += 4U) {
			/* There is only one receive FIFO, so use OTG_FS_FIFO(0) */
			(void)OTG_FS_FIFO(0U);
		}
		device->rxbcnt = 0U;

		/* If this is for a completion, re-arm the endpoint, preserving ACK state */
		if (phase == OTG_GRXSTSP_PKTSTS_SETUP_COMP || phase == OTG_GRXSTSP_PKTSTS_OUT_COMP) {
			OTG_FS_DOEPTSIZ(ep) = device->doeptsiz[ep];
			OTG_FS_DOEPCTL(ep) |= OTG_DOEPCTL0_EPENA | (device->force_nak[ep] ? OTG_DOEPCTL0_SNAK : OTG_DOEPCTL0_CNAK);
		} else
			OTG_FS_DOEPINT(ep) = OTG_DOEPINTX_XFRC;
	}

	/* Process suspend and wakeup interrupts */
	if (status & OTG_GINTSTS_USBSUSP) {
		if (device->user_callback_suspend)
			device->user_callback_suspend();
		OTG_FS_GINTSTS = OTG_GINTSTS_USBSUSP;
	}
	if (status & OTG_GINTSTS_WKUPINT) {
		if (device->user_callback_resume)
			device->user_callback_resume();
		OTG_FS_GINTSTS = OTG_GINTSTS_WKUPINT;
	}

	/* Handle SOF notifications */
	if (status & OTG_GINTSTS_SOF) {
		if (device->user_callback_sof)
			device->user_callback_sof();
		OTG_FS_GINTSTS = OTG_GINTSTS_SOF;
	}
}

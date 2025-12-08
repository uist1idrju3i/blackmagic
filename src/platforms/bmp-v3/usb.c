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

/* Receive FIFO size in 32-bit words. */
#define RX_FIFO_SIZE 32U /* 128 bytes */
/* FIFO access is made through pointers derived by this */
#define OTG_FS_FIFO(x) MMIO32(USB_OTG_FS_BASE + (0x1000U * (x + 1U)))

#define OTG_DCFG_ERRATIM (1U << 15U)

static usbd_device *bmd_dwc2_init(void);
void bmd_dwc2_disconnect(usbd_device *device, bool disconnect);
static void bmd_dwc2_set_address(usbd_device *device, uint8_t address);
static uint16_t bmd_dwc2_read_packet(usbd_device *device, uint8_t endpoint, void *buffer, uint16_t length);
static uint16_t bmd_dwc2_write_packet(usbd_device *device, uint8_t endpoint, const void *buffer, uint16_t length);
static void bmd_dwc2_stall_set(usbd_device *device, uint8_t endpoint, uint8_t stall);
static uint8_t bmd_dwc2_stall_get(usbd_device *device, uint8_t endpoint);
static void bmd_dwc2_nak_set(usbd_device *device, uint8_t endpoint, uint8_t nak);
static void bmd_dwc2_ep_setup(usbd_device *device, uint8_t endpoint_address, uint8_t type, uint16_t max_packet_length,
	void (*callback)(usbd_device *usbd_dev, uint8_t ep));
static void bmd_dwc2_endpoints_reset(usbd_device *device);
static void bmd_dwc2_poll(usbd_device *device);

static usbd_device usbd_dev;

const struct _usbd_driver bmd_dwc2_usb_driver = {
	.init = bmd_dwc2_init,
	.set_address = bmd_dwc2_set_address,
	.ep_setup = bmd_dwc2_ep_setup,
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
	/* Set up for USB operation on a 160MHz AHB, don't enable HNP, or SRP and force device mode */
	OTG_FS_GUSBCFG = OTG_GUSBCFG_FDMOD | (6U << 10U);
	/* Clear all outstanding interrupts so we're in a clean state */
	OTG_FS_GINTSTS = UINT32_MAX;
	/*
	 * Unmask interrupts for core events - SOF, RX FIFO non-empty, USB suspend, USB reset,
	 * enumeration done, IN and OUT endpoint interrupts, amd wake-up detected
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
	/* Interrupt when OUT transfer has completed SETUP phase */
	OTG_FS_DOEPMSK = OTG_DOEPMSK_STUPM;

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
	usbd_device *const device, const uint8_t endpoint, void *const buffer, const uint16_t length)
{
	/* We do not need to know the endpoint address since there is only one receive FIFO for all endpoints. */
	(void)endpoint;
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
	usbd_device *const device, const uint8_t endpoint, const void *const buffer, const uint16_t length)
{
	return 0U;
}

static void bmd_dwc2_stall_set(usbd_device *const device, const uint8_t endpoint, const uint8_t stall)
{
}

static uint8_t bmd_dwc2_stall_get(usbd_device *const device, const uint8_t endpoint)
{
	return true;
}

static void bmd_dwc2_nak_set(usbd_device *const device, const uint8_t endpoint, const uint8_t nak)
{
}

static void bmd_dwc2_ep_setup(usbd_device *const device, const uint8_t endpoint_address, const uint8_t type,
	const uint16_t max_packet_length, void (*const callback)(usbd_device *usbd_dev, uint8_t ep))
{
}

static void bmd_dwc2_endpoints_reset(usbd_device *const device)
{
}

static void bmd_dwc2_poll(usbd_device *const device)
{
	const uint32_t status = OTG_FS_GINTSTS;
	/* First check to see if we're here for a USB reset event */
	if (status & OTG_GINTSTS_USBRST) {
		/* Do an endpoint reset, make sure EP0 is set up, and clear the condition */
		device->fifo_mem_top = device->driver->rx_fifo_size;
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
}

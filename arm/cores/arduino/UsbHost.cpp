/*
 * Copyright 2009-2011 Oleg Mazurov, Circuits At Home, http://www.circuitsathome.com
 * Copyright (C) 2014 Manuel Stahl <thymythos@gmail.com>
 * MK20DX256 USB host controller support
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* USB functions */

#include "Arduino.h"
#include "usb_desc.h"
#include "kinetis.h"
#include <stdio.h>
#include "UsbHost.h"


#define BDT_OWN		0x80
#define BDT_DATA1	0x40
#define BDT_DATA0	0x00
#define BDT_DTS		0x08
#define BDT_STALL	0x04
#define BDT_LEN(n)	((n) >> 16)
#define BDT_READY(n)	!((n) & BDT_OWN)
#define BDT_PID(n)	(((n) >> 2) & 15)
#define BDT_PID_ACK		0x02
#define BDT_PID_DATA0	0x03
#define BDT_PID_NAK		0x0A
#define BDT_PID_DATA1	0x0B
#define BDT_PID_STALL	0x0E
#define BDT_PID_TIMEOUT	0x00
#define BDT_PID_ERROR	0x0F

#define BDT_DESC(count, data)	(BDT_OWN \
				| ((data) ? BDT_DATA1 : BDT_DATA0) \
				| ((count) << 16))

#define TX   1
#define RX   0
#define ODD  1
#define EVEN 0
#define DATA0 0
#define DATA1 1
#define index(tx) (((tx) << 1) | (bdt_odd[tx]))

// buffer descriptor table
typedef struct {
	uint32_t desc;
	void * addr;
} bdt_t;

__attribute__ ((section(".usbdescriptortable"), used))
static volatile bdt_t table[4]; // only pipe0 is used for all EPs
static bool bdt_odd[2];

static uint32_t usb_error = 0;
static uint32_t usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;


/**
 * \brief USBHost class constructor.
 */
USBHost::USBHost() : bmHubPre(0)
{
	unsigned i;
	// Set up state machine
	usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;

	for (i = 0; i < 4; i++) {
		table[i].desc = 0;
		table[i].addr = 0;
	}
	bdt_odd[RX] = 0;
	bdt_odd[TX] = 0;

	// this basically follows the flowchart in the Kinetis
	// Quick Reference User Guide, Rev. 1, 03/2012, page 141

	// assume 48 MHz clock already running
	// SIM - enable clock
	SIM_SCGC4 |= SIM_SCGC4_USBOTG;

	// reset USB module
	USB0_USBTRC0 = USB_USBTRC_USBRESET;
	while ((USB0_USBTRC0 & USB_USBTRC_USBRESET) != 0) ; // wait for reset to end

	// set desc table base addr
	USB0_BDTPAGE1 = ((uint32_t)table) >> 8;
	USB0_BDTPAGE2 = ((uint32_t)table) >> 16;
	USB0_BDTPAGE3 = ((uint32_t)table) >> 24;

	// clear all ISR flags
	USB0_ISTAT = 0xFF;
	USB0_ERRSTAT = 0xFF;
	USB0_OTGISTAT = 0xFF;

//	1. Enable Host Mode (CTL[HOST_MODE_EN]=1). The pull-down resistors are
//	enabled, and pull-up disabled. Start of Frame (SOF) generation begins. SOF counter
//	loaded with 12,000. Disable SOF packet generation to eliminate noise on the USB by
//	writing the USB enable bit to 0 (CTL[USB_EN]=0).
	USB0_CTL = USB_CTL_HOSTMODEEN;
	USB0_USBCTRL = 0;

	//	2. Enable the ATTACH interrupt (INT_ENB[ATTACH]=1).
	USB0_INTEN = USB_INTEN_ATTACHEN | USB_INTEN_USBRSTEN;

	// Init host stack
	init();
}

/**
 * \brief Initialize USBHost class.
 */
void USBHost::init()
{
	devConfigIndex	= 0;
	bmHubPre		= 0;
}


/**
 * \brief Get USBHost state.
 *
 * \return USB enumeration status (see USBHost::task).
 */
uint32_t USBHost::getUsbTaskState(void)
{
    return (usb_task_state);
}

/**
 * \brief Set USB state.
 *
 * \param state New USBHost status to be set.
 */
void USBHost::setUsbTaskState(uint32_t state)
{
    usb_task_state = state;
}

/**
 * \brief Get endpoint info from USB device address and device endpoint.
 *
 * \note This function should be used to know which host pipe is being used for
 * the corresponding device endpoint.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 *
 * \return Pointer to an EpInfo structure.
 */
EpInfo* USBHost::getEpInfoEntry(uint32_t addr, uint32_t ep)
{
	UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

	if (!p || !p->epinfo)
		return NULL;

	EpInfo *pep = p->epinfo;

	for (uint32_t i = 0; i < p->epcount; i++)
	{
		if (pep->deviceEpNum == ep)
			return pep;

		pep++;
	}

	return NULL;
}

/**
 * \brief Set device endpoint entry.
 *
 * \note Each device is different and has a different number of endpoints.
 * This function sets endpoint record structure to the device using address
 * addr in the address pool.
 *
 * \param ul_pipe Pipe address.
 * \param ul_token_type Token type.
 *
 * \retval 0 on success.
 * \retval USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL device not found.
 */
uint32_t USBHost::setEpInfoEntry(uint32_t addr, uint32_t epcount, EpInfo* eprecord_ptr)
{
	if (!eprecord_ptr)
		return USB_ERROR_INVALID_ARGUMENT;

	UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

	if (!p)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

	p->address	= addr;
	p->epinfo	= eprecord_ptr;
	p->epcount	= epcount;

	return 0;
}

/**
 * \brief Set host pipe target address and set ppep pointer to the endpoint
 * structure matching the specified USB device address and endpoint.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param ppep Endpoint info structure pointer set by setPipeAddress.
 * \param nak_limit Maximum number of NAK permitted.
 *
 * \retval 0 on success.
 * \retval USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL device not found.
 * \retval USB_ERROR_EPINFO_IS_NULL no endpoint structure found for this device.
 * \retval USB_ERROR_EP_NOT_FOUND_IN_TBL the specified device endpoint cannot be found.
 */
uint32_t USBHost::setPipeAddress(uint32_t addr, uint32_t ep, EpInfo **ppep, uint32_t &nak_limit)
{
	UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

	if (!p)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

 	if (!p->epinfo)
		return USB_ERROR_EPINFO_IS_NULL;

	*ppep = getEpInfoEntry(addr, ep);

	if (!*ppep)
		return USB_ERROR_EP_NOT_FOUND_IN_TBL;

	nak_limit = (0x0001UL << (((*ppep)->bmNakPower > USB_NAK_MAX_POWER ) ? USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
	nak_limit--;

	// Set peripheral address
	TRACE_USBHOST(Serial1.printf("     => SetAddress ep=%lu sending to address=%lu\r\n", ep, addr);)
	USB0_ADDR = (p->lowspeed ? 0x80 : 0x00) | (addr & 0x7f); // uhd_configure_address((*ppep)->hostPipeNum, addr);
	USB0_ENDPT0 = /* USB_ENDPT_RETRYDIS | */ USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
	// TODO set USB_ENDPT_HOSTWOHUB for EP0 when low-speed device without HUB connected

	return 0;
}

/**
 * \brief Send a control request.
 * Sets address, endpoint, fills control packet with necessary data, dispatches
 * control packet, and initiates bulk IN transfer depending on request.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param bmReqType Request direction.
 * \param bRequest Request type.
 * \param wValLo Value low.
 * \param wValHi Value high.
 * \param wInd Index field.
 * \param total Request length.
 * \param nbytes Number of bytes to read.
 * \param dataptr Data pointer.
 * \param p USB class reader.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::ctrlReq(uint32_t addr, uint32_t ep, uint8_t bmReqType, uint8_t bRequest, uint8_t wValLo, uint8_t wValHi, uint16_t wInd, uint16_t total, uint32_t nbytes, uint8_t* dataptr, USBReadParser *p)
{
	// Request direction, IN or OUT
	uint32_t direction = 0;
	uint32_t rcode = 0;
	SETUP_PKT setup_pkt;

	EpInfo *pep = 0;
	uint32_t nak_limit;

	TRACE_USBHOST(Serial1.printf("    => ctrlReq\r\n");)

	// Set peripheral address
	rcode = setPipeAddress(addr, ep, &pep, nak_limit);
	if (rcode)
		return rcode;

	// Determine request direction
	direction = ((bmReqType & 0x80 ) > 0);

	// Fill in setup packet
    setup_pkt.ReqType_u.bmRequestType	= bmReqType;
    setup_pkt.bRequest					= bRequest;
    setup_pkt.wVal_u.wValueLo			= wValLo;
    setup_pkt.wVal_u.wValueHi			= wValHi;
    setup_pkt.wIndex					= wInd;
    setup_pkt.wLength					= total;

	// Dispatch packet
	TRACE_USBHOST(Serial1.printf("    => setup\r\n");)
    pep->bmSndToggle = DATA0;
	rcode = dispatchPkt(USB_PID_SETUP, pep, &setup_pkt, sizeof(setup_pkt), nak_limit);
	if (rcode)
	{
		// Return HRSLT if not zero
		TRACE_USBHOST(Serial1.printf("/!\\ USBHost::ctrlReq : Setup packet error: %lu\r\n", rcode);)
		return (rcode);
	}

	// Data stage (if present)
	if (dataptr != 0)
	{
		if (direction)
		{
			// IN transfer
			TRACE_USBHOST(Serial1.printf("    => ctrlData IN (%lu bytes)\r\n", total);)
			uint32_t left = total;

			while (left)
			{
				// Bytes read into buffer
				uint32_t read = nbytes;

				rcode = InTransfer(pep, nak_limit, &read, dataptr);
				if (rcode)
					return rcode;

				TRACE_USBHOST(Serial1.printf("    got %lu of %lu bytes\r\n", read, left);)
				// Invoke callback function if inTransfer completed successfully and callback function pointer is specified
				if (!rcode && p)
					((USBReadParser*)p)->Parse(read, dataptr, total - left);

				if (read < left)
					left -= read;
				else
					left = 0;

				if (read != nbytes || left == 0)
					break;
			}
		}
		else
		{
			// OUT transfer
			TRACE_USBHOST(Serial1.printf("    => ctrlData OUT\r\n");)
			rcode = OutTransfer(pep, nak_limit, nbytes, dataptr);
		}

		if (rcode)
		{
			TRACE_USBHOST(Serial1.printf("/!\\ USBHost::ctrlData : Data packet error: %lu\r\n", rcode);)
			return (rcode);
		}
	}

	// Status stage
	TRACE_USBHOST(Serial1.printf("    => status\r\n", total);)
	if (direction) {
		return dispatchPkt(USB_PID_OUT, pep, nullptr, 0, nak_limit);
	} else {
		return dispatchPkt(USB_PID_IN, pep, nullptr, 0, nak_limit);
	}
}

/**
 * \brief Perform IN request to the specified USB device.
 *
 * \note This function handles multiple packets (if necessary) and can
 * receive a maximum of 'nbytesptr' bytes. It keep sending INs and writes data
 * to memory area pointed by 'data'. The actual amount of received bytes is
 * stored in 'nbytesptr'.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytesptr Receive buffer size. It is set to the amount of received
 * bytes when the function returns.
 * \param data Buffer to store received data.
 *
 * \return 0 on success, error code otherwise.
 */
 uint32_t USBHost::inTransfer(uint32_t addr, uint32_t ep, uint32_t *nbytesptr, uint8_t* data)
{
	EpInfo *pep = NULL;
	uint32_t nak_limit = 0;

	uint32_t rcode = setPipeAddress(addr, ep, &pep, nak_limit);

	if (rcode)
	{
		return rcode;
	}

	return InTransfer(pep, nak_limit, nbytesptr, data);
}

uint32_t USBHost::InTransfer(EpInfo *pep, uint32_t nak_limit, uint32_t *nbytesptr, uint8_t* data)
{
	uint32_t rcode = 0;
	uint32_t pktsize = 0;
	uint32_t nbytes = *nbytesptr;
	uint32_t maxpktsize = pep->maxPktSize;

	*nbytesptr = 0;

    while (1)
	{
    	unsigned idx = index(RX);

    	// Use a 'return' to exit this loop
		// IN packet to EP-'endpoint'. Function takes care of NAKS.
        rcode = dispatchPkt(USB_PID_IN, pep, data, maxpktsize, nak_limit);
        if (rcode)
		{
//			if (rcode == 1)
//			{
//				// TODO: Pipe freeze is mandatory to avoid sending IN endlessly (else reception becomes messy then)
//				uhd_freeze_pipe(pep->hostPipeNum);
//			}
			// Should be 1, indicating NAK. Else return error code.
            return rcode;
        }

		// Number of received bytes
		pktsize = BDT_LEN(table[idx].desc); // uhd_byte_count(pep->hostPipeNum);
		TRACE_USBHOST(Serial1.printf("      received %lu bytes\r\n", pktsize);)
		if (nbytes < pktsize)
		{
			TRACE_USBHOST(Serial1.printf("/!\\ USBHost::InTransfer : receive buffer is too small, size=%lu, expected=%lu\r\n", nbytes, pktsize);)
		}
        data += pktsize;

		// Add this packet's byte count to total transfer length
        *nbytesptr += pktsize;

        // The transfer is complete under two conditions:
        // 1. The device sent a short packet (L.T. maxPacketSize)
        // 2. 'nbytes' have been transferred.
        if ((pktsize < maxpktsize) || (*nbytesptr >= nbytes))
		{
            return 0;
        }
	}
}

/**
 * \brief Perform OUT request to the specified USB device.
 *
 * \note This function handles multiple packets (if necessary) and sends
 * 'nbytes' bytes.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size to be sent.
 * \param data Buffer to send.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::outTransfer(uint32_t addr, uint32_t ep, uint32_t nbytes, uint8_t* data)
{
	EpInfo *pep = NULL;
	uint32_t nak_limit = 0;

	uint32_t rcode = setPipeAddress(addr, ep, &pep, nak_limit);

	if (rcode)
	{
		return rcode;
	}

	return OutTransfer(pep, nak_limit, nbytes, data);
}

uint32_t USBHost::OutTransfer(EpInfo *pep, uint32_t nak_limit, uint32_t nbytes, uint8_t *data)
{
	uint32_t rcode = 0;
	uint32_t bytes_tosend = 0;
	uint32_t bytes_left = nbytes;
	uint32_t maxpktsize = pep->maxPktSize;

	if (maxpktsize < 1)
		return USB_ERROR_INVALID_MAX_PKT_SIZE;

	while (bytes_left)
	{
		bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;

		// Use a 'return' to exit this loop
		// OUT packet to EP-'endpoint'. Function takes care of NAKS.
		rcode = dispatchPkt(USB_PID_OUT, pep, data, bytes_tosend, nak_limit);
		if (rcode)
		{
			// Should be 0, indicating ACK. Else return error code.
			return rcode;
		}

		bytes_left -= bytes_tosend;
		data += bytes_tosend;
	}

	// Should be 0 in all cases
	return rcode;
}

/**
 * \brief Dispatch USB packet.
 *
 * \note Ensure peripheral address is set and relevant buffer is loaded/empty.
 * If NAK, tries to re-send up to nak_limit times.
 * If nak_limit == 0, do not count NAKs, exit after timeout.
 *
 * \param token Token type (Setup, In or Out).
 * \param toggle DATA0 or DATA1; toggled when ACK received.
 * \param ep Host endpoint number to use for sending USB packet.
 * \param nak_limit Maximum number of NAK permitted.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::dispatchPkt(uint8_t token, EpInfo *pep, void *data, uint32_t length, uint32_t nak_limit)
{
	uint32_t timeout = millis() + USB_XFER_TIMEOUT;
	uint32_t nak_count = 0;
	uint32_t rcode = USB_ERROR_TRANSFER_TIMEOUT;
	uint8_t tx = (token == USB_PID_IN) ? RX : TX;
    unsigned idx = index(tx);
	table[idx].desc = BDT_DESC(length, tx ? pep->bmSndToggle : 0);
	table[idx].addr = data;

	TRACE_USBHOST(Serial1.printf("     => dispatchPkt idx=%lu desc=0x%08X addr=0x%04X\r\n", idx, table[idx].desc, table[idx].addr);)
	TRACE_USBHOST(Serial1.printf("     => dispatchPkt token=0x%02X length=%lu nak_limit=%lu\r\n", token, length, nak_limit);)

	// Launch the transfer
	while (USB0_CTL & USB_CTL_TXSUSPENDTOKENBUSY) {
		TRACE_USBHOST(Serial1.printf("     token busy\r\n");)
		if (timeout > millis()) return rcode;
	}
	USB0_TOKEN = token | (pep->deviceEpNum & 0x0F); // UHD_Pipe_Send(hostPipeNum, token);
	bdt_odd[tx] = !bdt_odd[tx];

	// Check timeout but don't hold timeout if VBUS is lost
	while ((timeout > millis()) /* && (UHD_GetVBUSState() == UHD_STATE_CONNECTED) */ )
	{
		uint32_t desc = table[idx].desc;
		// Wait for transfer completion
		if (BDT_READY(desc))
		{
			uint8_t pid = BDT_PID(desc);
			USB0_ISTAT = 0xFF;
			// Is NAK received?
			switch (pid)
			{
			case BDT_PID_DATA0:
			case BDT_PID_DATA1:
			case BDT_PID_ACK:
				if (tx) pep->bmSndToggle = !pep->bmSndToggle;
				return 0;
			case BDT_PID_NAK:
				nak_count++;
				TRACE_USBHOST(Serial1.printf("     NAK\r\n");)

				if (nak_limit && (nak_count == nak_limit))
				{
					// Return NAK
					TRACE_USBHOST(Serial1.printf("     NAK limit\r\n");)
					return 1;
				}
				break;
			case BDT_PID_STALL:
			case BDT_PID_TIMEOUT:
			case BDT_PID_ERROR:
			default:
				TRACE_USBHOST(Serial1.printf("     token pid = 0x%X\r\n", pid);)
				TRACE_USBHOST(Serial1.printf("     stat=0x%02X istat=0x%02X errstat=0x%02X\r\n", USB0_STAT, USB0_ISTAT, USB0_ERRSTAT);)
				return 1;
			}
		}
	}

	TRACE_USBHOST(Serial1.printf("     TIMEOUT stat=0x%02X istat=0x%02X errstat=0x%02X\r\n", USB0_STAT, USB0_ISTAT, USB0_ERRSTAT);)
	return rcode;
}

/**
 * \brief Configure device using known device classes.
 * The device get a new address even if its class remain unknown.
 *
 * \param parent USB device address of the device's parent (0 if root).
 * \param port USB device base address (see AddressPoolImpl).
 * \param lowspeed Device speed.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::Configuring(uint32_t parent, uint32_t port, uint32_t lowspeed)
{
	uint32_t rcode = 0;

	for (; devConfigIndex < USB_NUMDEVICES; ++devConfigIndex)
	{
		if (!devConfig[devConfigIndex])
			continue;

		rcode = devConfig[devConfigIndex]->Init(parent, port, lowspeed);

		if (!rcode)
		{
			TRACE_USBHOST(Serial1.printf("USBHost::Configuring : found device class!\r\n");)
			devConfigIndex = 0;
			return 0;
		}


		if (rcode == USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED)
		{
			TRACE_USBHOST(Serial1.printf("USBHost::Configuring : ERROR : device not supported!\r\n");)
		}
		else if (rcode == USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE)
		{
			TRACE_USBHOST(Serial1.printf("USBHost::Configuring : ERROR : class instance already in use!\r\n");)
		}
		else
		{
			// in case of an error devConfigIndex should be reset to 0
			// in order to start from the very beginning the next time
			// the program gets here
			if (rcode != USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE)
				devConfigIndex = 0;

			return rcode;
		}
	}

	// Device class is not supported by any of the registered classes
	devConfigIndex = 0;

	rcode = DefaultAddressing(parent, port, lowspeed);

	return rcode;
}

/**
 * \brief Configure device with unknown USB class.
 *
 * \param parent USB device address of the device's parent (0 if root).
 * \param port USB device base address (see AddressPoolImpl).
 * \param lowspeed Device speed.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::DefaultAddressing(uint32_t parent, uint32_t port, uint32_t lowspeed)
{
	uint32_t rcode = 0;
	UsbDevice *p0 = 0, *p = 0;

	// Get pointer to pseudo device with address 0 assigned
	p0 = addrPool.GetUsbDevicePtr(0);

	if (!p0)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

	if (!p0->epinfo)
		return USB_ERROR_EPINFO_IS_NULL;

	p0->lowspeed = (lowspeed) ? 1 : 0;

	// Allocate new address according to device class
	uint32_t bAddress = addrPool.AllocAddress(parent, 0, port);

	if (!bAddress)
		return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

	p = addrPool.GetUsbDevicePtr(bAddress);

	if (!p)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

	p->lowspeed = lowspeed;

	// Assign new address to the device
	rcode = setAddr(0, 0, bAddress);

	if (rcode)
	{
		TRACE_USBHOST(Serial1.printf("/!\\ USBHost::DefaultAddressing : Set address failed with code: %lu\r\n", rcode);)
		addrPool.FreeAddress(bAddress);
		bAddress = 0;
		return rcode;
	}

	return 0;
}

/**
 * \brief Release device and free associated resources.
 *
 * \param addr USB device address.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::ReleaseDevice(uint32_t addr)
{
	if (!addr)
		return 0;

	for (uint32_t i = 0; i < USB_NUMDEVICES; ++i)
	{
		if (devConfig[i]->GetAddress() == addr)
		{
			return devConfig[i]->Release();
		}
	}

	return 0;
}

/**
 * \brief Get device descriptor.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size.
 * \param dataptr Buffer to store received descriptor.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getDevDescr(uint32_t addr, uint32_t ep, uint32_t nbytes, uint8_t* dataptr)
{
    return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00, USB_DESCRIPTOR_DEVICE, 0x0000, nbytes, nbytes, dataptr, 0));
}

/**
 * \brief Get configuration descriptor.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size.
 * \param conf Configuration number.
 * \param dataptr Buffer to store received descriptor.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getConfDescr(uint32_t addr, uint32_t ep, uint32_t nbytes, uint32_t conf, uint8_t* dataptr)
{
	return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, nbytes, nbytes, dataptr, 0));
}

/**
 * \brief Get configuration descriptor and extract endpoints using USBReadParser object.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param conf Configuration number.
 * \param p USBReadParser object pointer used to extract endpoints.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getConfDescr(uint32_t addr, uint32_t ep, uint32_t conf, USBReadParser *p)
{
	const uint32_t bufSize = 64;
	uint8_t buf[bufSize];

	uint32_t ret = getConfDescr(addr, ep, 8, conf, buf);

	if (ret)
		return ret;

	uint32_t total = ((USB_CONFIGURATION_DESCRIPTOR*)buf)->wTotalLength;
	delay(100);

    return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, total, bufSize, buf, p));
}

/**
 * \brief Get string descriptor.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size.
 * \param index String index.
 * \param langid Language ID.
 * \param dataptr Buffer to store received descriptor.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getStrDescr(uint32_t addr, uint32_t ep, uint32_t nbytes, uint8_t index, uint16_t langid, uint8_t* dataptr)
{
    return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, langid, nbytes, nbytes, dataptr, 0));
}

/**
 * \brief Set USB device address.
 *
 * \param oldaddr Current USB device address.
 * \param ep USB device endpoint number.
 * \param addr New USB device address to be set.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::setAddr(uint32_t oldaddr, uint32_t ep, uint32_t newaddr)
{
	TRACE_USBHOST(Serial1.printf("   => USBHost::setAddr to %lu\r\n", newaddr);)
    return ctrlReq(oldaddr, ep, bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00, 0x0000, 0x0000, 0x0000, 0, 0);
}

/**
 * \brief Set configuration.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param conf_value New configuration value to be set.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::setConf(uint32_t addr, uint32_t ep, uint32_t conf_value)
{
    return (ctrlReq(addr, ep, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000, 0x0000, 0x0000, 0, 0));
}

/**
 * \brief USB main task, responsible for enumeration and clean up stage.
 *
 * \note Must be periodically called from loop().
 */
void USBHost::Task(void)
{
	uint32_t rcode = 0;
	static uint32_t delay = 0;
	uint32_t lowspeed = 0;

    // Update USB task state
    if (USB0_ISTAT & USB_ISTAT_ERROR) {
		// Illegal state
		usb_task_state = USB_DETACHED_SUBSTATE_ILLEGAL;
		lowspeed = 0;
		USB0_ISTAT |= USB_ISTAT_ERROR;
	}

	if (USB0_ISTAT & USB_ISTAT_ATTACH) {
		// Attached state
		if ((usb_task_state & USB_STATE_MASK) == USB_STATE_DETACHED ||
				usb_task_state == USB_STATE_ERROR)
		{
			delay = millis() + USB_SETTLE_DELAY;
			usb_task_state = USB_ATTACHED_SUBSTATE_SETTLE;
			lowspeed = !(USB0_CTL & USB_CTL_JSTATE);
			TRACE_USBHOST(Serial1.printf(" + USB_ATTACHED_SUBSTATE_SETTLE\r\n");)
		}
		USB0_ISTAT |= USB_ISTAT_ATTACH;
	}

	// Poll connected devices (if required)
	for (uint32_t i = 0; i < USB_NUMDEVICES; ++i)
		if (devConfig[i])
			rcode = devConfig[i]->Poll();

	// Perform USB enumeration stage and clean up
    switch (usb_task_state)
	{
        case USB_DETACHED_SUBSTATE_INITIALIZE:
			TRACE_USBHOST(Serial1.printf(" + USB_DETACHED_SUBSTATE_INITIALIZE\r\n");)

			// Init USB stack and driver
            init();

			// Free all USB resources
			for (uint32_t i = 0; i < USB_NUMDEVICES; ++i)
				if (devConfig[i])
					rcode = devConfig[i]->Release();

            usb_task_state = USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE;
            break;

        case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE:
			// Nothing to do
            break;

        case USB_DETACHED_SUBSTATE_ILLEGAL:
			// Nothing to do
            break;

        case USB_ATTACHED_SUBSTATE_SETTLE:
			// Settle time for just attached device
            if (delay < millis())
			{
				TRACE_USBHOST(Serial1.printf(" + USB_ATTACHED_SUBSTATE_SETTLE\r\n");)
                usb_task_state = USB_ATTACHED_SUBSTATE_RESET_DEVICE;
            }
            break;

        case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
			TRACE_USBHOST(Serial1.printf(" + USB_ATTACHED_SUBSTATE_RESET_DEVICE\r\n");)

			// Trigger Bus Reset
            USB0_CTL |= USB_CTL_RESET;
            usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE;
            delay = millis() + 10;
            break;

        case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
            if (delay < millis())
			{
				TRACE_USBHOST(Serial1.printf(" + USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE\r\n");)

				// Clear Bus Reset flag
				USB0_CTL &= ~USB_CTL_RESET;

				// Enable Start Of Frame generation
				USB0_CTL |= USB_CTL_USBENSOFEN;

                usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_SOF;

				// Wait 20ms after Bus Reset (USB spec)
                delay = millis() + 20;
            }
            break;

        case USB_ATTACHED_SUBSTATE_WAIT_SOF:
			// Wait for SOF received first
            if (USB0_ISTAT & USB_ISTAT_SOFTOK)
			{
				if (delay < millis())
				{
					TRACE_USBHOST(Serial1.printf(" + USB_ATTACHED_SUBSTATE_WAIT_SOF\r\n");)

					// 20ms waiting elapsed
					usb_task_state = USB_STATE_CONFIGURING;
				}
            }
            break;

        case USB_STATE_CONFIGURING:
			TRACE_USBHOST(Serial1.printf(" + USB_STATE_CONFIGURING\r\n");)
			rcode = Configuring(0, 0, lowspeed);

			if (rcode)
			{
				TRACE_USBHOST(Serial1.printf("/!\\ USBHost::Task : USB_STATE_CONFIGURING failed with code: %lu\r\n", rcode);)
				if (rcode != USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE)
				{
					usb_error = rcode;
					usb_task_state = USB_STATE_ERROR;
				}
			}
			else
			{
				usb_task_state = USB_STATE_RUNNING;
				TRACE_USBHOST(Serial1.printf(" + USB_STATE_RUNNING\r\n");)
			}
            break;

        case USB_STATE_RUNNING:
            break;

        case USB_STATE_ERROR:
            break;
    }
}

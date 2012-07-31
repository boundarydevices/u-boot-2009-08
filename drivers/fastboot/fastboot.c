/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <common.h>
#include <fastboot.h>
#include <mmc.h>
#include <sata.h>
#include <usb/imx_udc.h>
#include <usbdevice.h>

/*
 * Defines
 */
#define NUM_ENDPOINTS  2

#define CONFIG_USBD_OUT_PKTSIZE	    0x200
#define CONFIG_USBD_IN_PKTSIZE	    0x200
#define MAX_BUFFER_SIZE		    0x200

#define STR_LANG_INDEX		    0x00
#define STR_MANUFACTURER_INDEX	    0x01
#define STR_PRODUCT_INDEX	    0x02
#define STR_SERIAL_INDEX	    0x03
#define STR_CONFIG_INDEX	    0x04
#define STR_DATA_INTERFACE_INDEX    0x05
#define STR_CTRL_INTERFACE_INDEX    0x06
#define STR_COUNT		    0x07


/* defined and used by gadget/ep0.c */
extern struct usb_string_descriptor **usb_strings;

static struct usb_device_instance device_instance[1];
static struct usb_bus_instance bus_instance[1];
static struct usb_configuration_instance config_instance[1];
static struct usb_interface_instance interface_instance[1];
static struct usb_alternate_instance alternate_instance[1];
/* one extra for control endpoint */
struct usb_endpoint_instance endpoint_instance[NUM_ENDPOINTS+1];

static struct cmd_fastboot_interface *fastboot_interface;
static int fastboot_configured_flag;
static int connected;

/* Indicies, References */
static u8 rx_endpoint;
u8 tx_endpoint;
static struct usb_string_descriptor *fastboot_string_table[STR_COUNT];

/* USB Descriptor Strings */
static u8 wstrLang[4] = {4, USB_DT_STRING, 0x9, 0x4};
static u8 wstrManufacturer[2 * (sizeof(CONFIG_FASTBOOT_MANUFACTURER_STR))];
static u8 wstrProduct[2 * (sizeof(CONFIG_FASTBOOT_PRODUCT_NAME_STR))];
static u8 wstrSerial[2*(sizeof(CONFIG_FASTBOOT_SERIAL_NUM))];
static u8 wstrConfiguration[2 * (sizeof(CONFIG_FASTBOOT_CONFIGURATION_STR))];
static u8 wstrDataInterface[2 * (sizeof(CONFIG_FASTBOOT_INTERFACE_STR))];

/* Standard USB Data Structures */
static struct usb_interface_descriptor interface_descriptors[1];
static struct usb_endpoint_descriptor *ep_descriptor_ptrs[NUM_ENDPOINTS];
static struct usb_configuration_descriptor *configuration_descriptor;
static struct usb_device_descriptor device_descriptor = {
	.bLength = sizeof(struct usb_device_descriptor),
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		cpu_to_le16(USB_BCD_VERSION),
	.bDeviceClass =		0xff,
	.bDeviceSubClass =	0xff,
	.bDeviceProtocol =	0xff,
	.bMaxPacketSize0 =	0x40,
	.idVendor =		cpu_to_le16(CONFIG_FASTBOOT_VENDOR_ID),
	.idProduct =		cpu_to_le16(CONFIG_FASTBOOT_PRODUCT_ID),
	.bcdDevice =		cpu_to_le16(CONFIG_FASTBOOT_BCD_DEVICE),
	.iManufacturer =	STR_MANUFACTURER_INDEX,
	.iProduct =		STR_PRODUCT_INDEX,
	.iSerialNumber =	STR_SERIAL_INDEX,
	.bNumConfigurations =	1
};

/*
 * Static Generic Serial specific data
 */

struct fastboot_config_desc {
	struct usb_configuration_descriptor configuration_desc;
	struct usb_interface_descriptor	interface_desc[1];
	struct usb_endpoint_descriptor data_endpoints[NUM_ENDPOINTS];

} __attribute__((packed));

static struct fastboot_config_desc
fastboot_configuration_descriptors[1] = {
	{
		.configuration_desc = {
			.bLength = sizeof(struct usb_configuration_descriptor),
			.bDescriptorType = USB_DT_CONFIG,
			.wTotalLength =
			    cpu_to_le16(sizeof(struct fastboot_config_desc)),
			.bNumInterfaces = 1,
			.bConfigurationValue = 1,
			.iConfiguration = STR_CONFIG_INDEX,
			.bmAttributes =
				BMATTRIBUTE_SELF_POWERED|BMATTRIBUTE_RESERVED,
			.bMaxPower = 0x32
		},
		.interface_desc = {
			{
				.bLength  =
					sizeof(struct usb_interface_descriptor),
				.bDescriptorType = USB_DT_INTERFACE,
				.bInterfaceNumber = 0,
				.bAlternateSetting = 0,
				.bNumEndpoints = NUM_ENDPOINTS,
				.bInterfaceClass =
					FASTBOOT_INTERFACE_CLASS,
				.bInterfaceSubClass =
					FASTBOOT_INTERFACE_SUB_CLASS,
				.bInterfaceProtocol =
					FASTBOOT_INTERFACE_PROTOCOL,
				.iInterface = STR_DATA_INTERFACE_INDEX
			},
		},
		.data_endpoints  = {
			{
				.bLength =
					sizeof(struct usb_endpoint_descriptor),
				.bDescriptorType =  USB_DT_ENDPOINT,
				.bEndpointAddress = UDC_OUT_ENDPOINT |
							 USB_DIR_OUT,
				.bmAttributes =	USB_ENDPOINT_XFER_BULK,
				.wMaxPacketSize =
					cpu_to_le16(CONFIG_USBD_OUT_PKTSIZE),
				.bInterval = 0x00,
			},
			{
				.bLength =
					sizeof(struct usb_endpoint_descriptor),
				.bDescriptorType =  USB_DT_ENDPOINT,
				.bEndpointAddress = UDC_IN_ENDPOINT |
							USB_DIR_IN,
				.bmAttributes =	USB_ENDPOINT_XFER_BULK,
				.wMaxPacketSize =
					cpu_to_le16(CONFIG_USBD_IN_PKTSIZE),
				.bInterval = 0x00,
			},
		},
	},
};

/* Static Function Prototypes */
static void fastboot_init_strings(void);
static void fastboot_init_instances(void);
static void fastboot_event_handler(struct usb_device_instance *device,
				usb_device_event_t event, int data);
static int fastboot_cdc_setup(struct usb_device_request *request,
				struct urb *urb);
static int fastboot_usb_configured(void);

/* utility function for converting char* to wide string used by USB */
static void str2wide(char *str, u16 * wide)
{
	int i;
	for (i = 0; i < strlen(str) && str[i]; i++) {
		#if defined(__LITTLE_ENDIAN)
			wide[i] = (u16) str[i];
		#elif defined(__BIG_ENDIAN)
			wide[i] = ((u16)(str[i])<<8);
		#else
			#error "__LITTLE_ENDIAN or __BIG_ENDIAN undefined"
		#endif
	}
}

/*
 * Initialize fastboot
 */
int fastboot_init(struct cmd_fastboot_interface *interface)
{
	printf("fastboot is in init......\n");
	connected = 0;
	fastboot_configured_flag = 0;
	fastboot_interface = interface;
	fastboot_interface->product_name = CONFIG_FASTBOOT_PRODUCT_NAME_STR;
	fastboot_interface->serial_no = CONFIG_FASTBOOT_SERIAL_NUM;
	fastboot_interface->nand_block_size = 4096;
	fastboot_interface->transfer_buffer =
				(unsigned char *)CONFIG_FASTBOOT_TRANSFER_BUF;
	fastboot_interface->transfer_buffer_size =
				CONFIG_FASTBOOT_TRANSFER_BUF_SIZE;
	fastboot_init_strings();
	/* Basic USB initialization */
	usb_shutdown();
	udc_init();

	fastboot_init_instances();
#ifdef CONFIG_FASTBOOT_STORAGE_SF
	fastboot_init_sf_ptable();
#endif
#ifdef CONFIG_FASTBOOT_STORAGE_EMMC_SATA
	fastboot_init_mmc_sata_ptable();
#endif
	fastboot_flash_dump_ptn();
	udc_startup_events(device_instance);
	return 0;
}

static void fastboot_init_strings(void)
{
	struct usb_string_descriptor *string;

	fastboot_string_table[STR_LANG_INDEX] =
		(struct usb_string_descriptor *)wstrLang;

	string = (struct usb_string_descriptor *)wstrManufacturer;
	string->bLength = sizeof(wstrManufacturer);
	string->bDescriptorType = USB_DT_STRING;
	str2wide(CONFIG_FASTBOOT_MANUFACTURER_STR, string->wData);
	fastboot_string_table[STR_MANUFACTURER_INDEX] = string;

	string = (struct usb_string_descriptor *)wstrProduct;
	string->bLength = sizeof(wstrProduct);
	string->bDescriptorType = USB_DT_STRING;
	str2wide(CONFIG_FASTBOOT_PRODUCT_NAME_STR, string->wData);
	fastboot_string_table[STR_PRODUCT_INDEX] = string;

	string = (struct usb_string_descriptor *)wstrSerial;
	string->bLength = sizeof(wstrSerial);
	string->bDescriptorType = USB_DT_STRING;
	str2wide(CONFIG_FASTBOOT_SERIAL_NUM, string->wData);
	fastboot_string_table[STR_SERIAL_INDEX] = string;

	string = (struct usb_string_descriptor *)wstrConfiguration;
	string->bLength = sizeof(wstrConfiguration);
	string->bDescriptorType = USB_DT_STRING;
	str2wide(CONFIG_FASTBOOT_CONFIGURATION_STR, string->wData);
	fastboot_string_table[STR_CONFIG_INDEX] = string;

	string = (struct usb_string_descriptor *) wstrDataInterface;
	string->bLength = sizeof(wstrDataInterface);
	string->bDescriptorType = USB_DT_STRING;
	str2wide(CONFIG_FASTBOOT_INTERFACE_STR, string->wData);
	fastboot_string_table[STR_DATA_INTERFACE_INDEX] = string;

	/* Now, initialize the string table for ep0 handling */
	usb_strings = fastboot_string_table;
}

static void fastboot_init_instances(void)
{
	int i;

	/* Assign endpoint descriptors */
	ep_descriptor_ptrs[0] =
		&fastboot_configuration_descriptors[0].data_endpoints[0];
	ep_descriptor_ptrs[1] =
		&fastboot_configuration_descriptors[0].data_endpoints[1];

	/* Configuration Descriptor */
	configuration_descriptor =
		&fastboot_configuration_descriptors[0].configuration_desc;

	/* initialize device instance */
	memset(device_instance, 0, sizeof(struct usb_device_instance));
	device_instance->device_state = STATE_INIT;
	device_instance->device_descriptor = &device_descriptor;
	device_instance->event = fastboot_event_handler;
	device_instance->cdc_recv_setup = fastboot_cdc_setup;
	device_instance->bus = bus_instance;
	device_instance->configurations = 1;
	device_instance->configuration_instance_array = config_instance;

	/* initialize bus instance */
	memset(bus_instance, 0, sizeof(struct usb_bus_instance));
	bus_instance->device = device_instance;
	bus_instance->endpoint_array = endpoint_instance;
	bus_instance->max_endpoints = NUM_ENDPOINTS + 1;
	bus_instance->maxpacketsize = 64;
	bus_instance->serial_number_str = CONFIG_FASTBOOT_SERIAL_NUM;

	/* configuration instance */
	memset(config_instance, 0,
		sizeof(struct usb_configuration_instance));
	config_instance->interfaces = 1;
	config_instance->configuration_descriptor = configuration_descriptor;
	config_instance->interface_instance_array = interface_instance;

	/* interface instance */
	memset(interface_instance, 0,
		sizeof(struct usb_interface_instance));
	interface_instance->alternates = 1;
	interface_instance->alternates_instance_array = alternate_instance;

	/* alternates instance */
	memset(alternate_instance, 0,
		sizeof(struct usb_alternate_instance));
	alternate_instance->interface_descriptor = interface_descriptors;
	alternate_instance->endpoints = NUM_ENDPOINTS;
	alternate_instance->endpoints_descriptor_array = ep_descriptor_ptrs;

	/* endpoint instances */
	memset(&endpoint_instance[0], 0,
		sizeof(struct usb_endpoint_instance));
	endpoint_instance[0].endpoint_address = 0;
	endpoint_instance[0].rcv_packetSize = EP0_MAX_PACKET_SIZE;
	endpoint_instance[0].rcv_attributes = USB_ENDPOINT_XFER_CONTROL;
	endpoint_instance[0].tx_packetSize = EP0_MAX_PACKET_SIZE;
	endpoint_instance[0].tx_attributes = USB_ENDPOINT_XFER_CONTROL;
	udc_setup_ep(device_instance, 0, &endpoint_instance[0]);

	for (i = 1; i <= NUM_ENDPOINTS; i++) {
		memset(&endpoint_instance[i], 0,
			sizeof(struct usb_endpoint_instance));

		endpoint_instance[i].endpoint_address =
			ep_descriptor_ptrs[i - 1]->bEndpointAddress;

		endpoint_instance[i].rcv_attributes =
			ep_descriptor_ptrs[i - 1]->bmAttributes;

		endpoint_instance[i].rcv_packetSize =
			le16_to_cpu(ep_descriptor_ptrs[i - 1]->wMaxPacketSize);

		endpoint_instance[i].tx_attributes =
			ep_descriptor_ptrs[i - 1]->bmAttributes;

		endpoint_instance[i].tx_packetSize =
			le16_to_cpu(ep_descriptor_ptrs[i - 1]->wMaxPacketSize);

		endpoint_instance[i].tx_attributes =
			ep_descriptor_ptrs[i - 1]->bmAttributes;

		urb_link_init(&endpoint_instance[i].rcv);
		urb_link_init(&endpoint_instance[i].rx_free);
		urb_link_init(&endpoint_instance[i].tx_ready);
		urb_link_init(&endpoint_instance[i].tx_free);

		if (endpoint_instance[i].endpoint_address & USB_DIR_IN) {
			tx_endpoint = i;
		} else {
			rx_endpoint = i;
			endpoint_instance[i].rcv_urb =
				usbd_alloc_urb(device_instance,
						&endpoint_instance[i]);
		}
	}
}

static void fastboot_init_endpoints(void)
{
	int i;

	bus_instance->max_endpoints = NUM_ENDPOINTS + 1;

	for (i = 1; i <= NUM_ENDPOINTS; i++) {
		struct usb_endpoint_instance *epi = &endpoint_instance[i];
		udc_setup_ep(device_instance, i, epi);
	}
}

static void fastboot_fix_max_packet_size(void)
{
	int i;
	int max = (usb_get_port_speed() == 2) ? 512 : 64;

	ep_descriptor_ptrs[0]->wMaxPacketSize = max;
	ep_descriptor_ptrs[1]->wMaxPacketSize = max;

	for (i = 1; i <= NUM_ENDPOINTS; i++) {
		struct usb_endpoint_instance *epi = &endpoint_instance[i];
		epi->tx_packetSize = max;
		epi->rcv_packetSize = max;
	}
}

int fastboot_get_rx_buffer(u8 *buf, int max)
{
	int byte_cnt = 0;
	struct usb_endpoint_instance *epi =
			&endpoint_instance[rx_endpoint];

	while (epi->rcv_urb && epi->rcv_urb->actual_length) {
		unsigned int nb = epi->rcv_urb->actual_length;
		char *src = (char *)epi->rcv_urb->buffer;

		if (nb > max) {
			if (byte_cnt)
				break;
			if (buf)
				memcpy(buf, src, max);
			printf("%s: returning %s", __func__, src);
			src += max;
			printf("%s: dropped %s", __func__, src);
			return nb;
		}
		if (buf)
			memcpy(buf, src, nb);
		epi->rcv_urb->actual_length = 0;

		byte_cnt += nb;
		buf += nb;
		max -= nb;
		/*
		 * buffer done if short packet received or exact amount
		 */
		if (!max || (nb & (epi->rcv_packetSize - 1))) {
			usb_rcv_urb(epi, epi->rcv_packetSize);
			break;
		}
		usb_rcv_urb(epi, max);
		if (byte_cnt >= (4096 * 32))
			break;
	}
	return byte_cnt;
}

static struct urb *next_tx_urb(struct usb_device_instance *device,
				struct usb_endpoint_instance *ep)
{
	struct urb *urb;

	if (ep->tx_ready.prev != ep->tx_ready.next) {
		/* > 1 buffer queued */
		urb = p2surround(struct urb, link, ep->tx_ready.prev);
		if (urb->actual_length < urb->buffer_length)
			return urb;
	}
	urb = first_urb_detached(&ep->tx_free);
	if (!urb)
		urb = usbd_alloc_urb(device, ep);
	if (urb)
		urb_append(&ep->tx_ready, urb);
	return urb;
}

int fastboot_getvar(const char *rx_buffer, char *tx_buffer)
{
	/* Place board specific variables here */
	return 0;
}

int fastboot_poll()
{
	/* If usb disconnected, blocked here to wait */
	if (is_usb_disconnected()) {
		udc_disconnect();
		fastboot_configured_flag = 0;
		if (connected) {
			connected = 0;
			printf("disconnected\n");
			return FASTBOOT_DISCONNECT;
		}
		return FASTBOOT_INACTIVE;
	}
	if (!connected) {
		connected = 1;
		printf("connected\n");
		udc_connect();		/* Enable pullup for host detection */
	}
	udc_irq();
	if (!fastboot_usb_configured())
		return FASTBOOT_INACTIVE;

	/* Pass this up to the interface's handler */
	if (fastboot_interface && fastboot_interface->rx_handler) {
		struct usb_endpoint_instance *epi =
				&endpoint_instance[rx_endpoint];
		if (epi->rcv_urb && epi->rcv_urb->actual_length) {
			fastboot_interface->rx_handler(fastboot_interface);
			return FASTBOOT_OK;
		}
	}
	return FASTBOOT_INACTIVE;
}

int fastboot_tx(unsigned char *buffer, unsigned int buffer_size)
{
	/* Not realized yet */
	return 0;
}

static int write_buffer(const char *buffer, unsigned int buffer_size)
{
	struct usb_endpoint_instance *epi =
		(struct usb_endpoint_instance *)&endpoint_instance[tx_endpoint];
	struct urb *current_urb = NULL;

	if (!fastboot_usb_configured())
		return 0;

	current_urb = next_tx_urb(device_instance, epi);
	if (buffer_size) {
		char *dest;
		int space_avail, popnum, count, total = 0;

		/* Break buffer into urb sized pieces,
		 * and link each to the endpoint
		 */
		count = buffer_size;
		while (count > 0) {
			if (!current_urb) {
				printf("current_urb is NULL, buffer_size %d\n",
				    buffer_size);
				return total;
			}

			dest = (char *)current_urb->buffer +
			current_urb->actual_length;

			space_avail = current_urb->buffer_length -
					current_urb->actual_length;
			popnum = MIN(space_avail, count);
			if (popnum == 0)
				break;

			if (0)
				printf("actual_length=%x popnum=%x space_avail=%x, count=%x\n",
					current_urb->actual_length, popnum, space_avail, count);
			memcpy(dest, buffer + total, popnum);
			if (0)
				printf("send: %s\n", (char *)buffer);

			current_urb->actual_length += popnum;
			total += popnum;

			if (udc_endpoint_write(epi)) {
				/* Write pre-empted by RX */
				printf("rx preempt\n");
				return 0;
			}
			count -= popnum;
		} /* end while */
		return total;
	}
	return 0;
}

int fastboot_tx_status(const char *buffer, unsigned int buffer_size)
{
	int len = 0;

	while (buffer_size > 0) {
		len = write_buffer(buffer + len, buffer_size);
		buffer_size -= len;

		udc_irq();
	}
	udc_irq();

	return 0;
}

void free_urb_list(struct urb_link *urb_link)
{
	struct urb *urb;
	for (;;) {
		urb = first_urb_detached(urb_link);
		if (!urb)
			return;
		usbd_dealloc_urb(urb);
	}
}

void fastboot_shutdown(void)
{
	struct usb_endpoint_instance *epi = &endpoint_instance[0];
	int i;
	usb_shutdown();

	for(i = 0; i < ARRAY_SIZE(endpoint_instance); i++, epi++) {
		free_urb_list(&epi->rcv);	/* received urbs */
		free_urb_list(&epi->rx_free);	/* empty urbs ready to receive */
		usbd_dealloc_urb(epi->rcv_urb);	/* active urb */
		free_urb_list(&epi->tx_ready);	/* urbs ready to transmit */
		free_urb_list(&epi->tx_free);	/* transmitted urbs */
	}
	/* Reset some globals */
	fastboot_interface = NULL;
}

static int fastboot_usb_configured(void)
{
	return fastboot_configured_flag;
}

static void fastboot_event_handler(struct usb_device_instance *device,
				  usb_device_event_t event, int data)
{
	switch (event) {
	case DEVICE_RESET:
	case DEVICE_BUS_INACTIVE:
		fastboot_configured_flag = 0;
		break;
	case DEVICE_CONFIGURED:
		fastboot_configured_flag = 1;
		fastboot_init_endpoints();
		break;
	case DEVICE_ADDRESS_ASSIGNED:
		fastboot_fix_max_packet_size();
	default:
		break;
	}
}

int fastboot_cdc_setup(struct usb_device_request *request, struct urb *urb)
{
	return 0;
}

/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc.
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
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/types.h>
#include <malloc.h>
#include <usbdevice.h>
#include <usb/imx_udc.h>

#include "ep0.h"
//#define DEBUG
#ifdef DEBUG
#define DBG(x...) printf(x)
#define ENTRY printf("%s:entry\n", __func__);
#define EXIT(a) printf("%s:exit %x\n", __func__, a);
#else
#define DBG(x...) do {} while (0)
#define ENTRY
#define EXIT(a)
#endif


#define mdelay(n) udelay((n)*1000)

#define EP_TQ_ITEM_CNT 8	/* keep power of 2 */

#define inc_index(x) ((x+1) % EP_TQ_ITEM_CNT)


#define USB_RECIP_MASK	    0x03
#define USB_TYPE_MASK	    (0x03 << 5)
#define USB_MEM_ALIGN_BYTE  4096

typedef struct {
	int epnum;
	int dir;
	int max_pkt_size;
	struct usb_endpoint_instance *epi;
	struct ep_queue_item *ep_dtd[EP_TQ_ITEM_CNT]; /* malloced dmaable */
	void *release_dtd;
	void *release_payload;
	int index; /* to index the free tx tqi */
	int done;  /* to index the complete rx tqi */
	struct ep_queue_item *tail; /* last item in the dtd chain */
	struct ep_queue_head *ep_qh;
} mxc_ep_t;

typedef struct {
	int	max_ep;
	int	setaddr;
	struct ep_queue_head *ep_qh;	/* malloced dmaable, array of max_ep entries */
	void	*release;
	mxc_ep_t *mxc_ep;		/* malloced array of max_ep entries */
	u32	qh_dma;
	struct urb *ep0_urb;
} mxc_udc_ctrl;

static int usb_highspeed;
static int usb_inited;
static mxc_udc_ctrl mxc_udc;
static struct usb_device_instance *udc_device;

static void release_usb_space(void)
{
	mxc_ep_t *ep = mxc_udc.mxc_ep;

	usbd_dealloc_urb(mxc_udc.ep0_urb);
	mxc_udc.ep0_urb = NULL;
	if (ep) {
		/*
		 * release in reverse order of allocation
		 * so that same buffer is returned
		 * next allocation, to ease debugging
		 */
		int i = mxc_udc.max_ep - 1;
		ep += i;
		for (; i >= 0; i--, ep--) {
			free(ep->release_payload);
			ep->release_payload = NULL;
			free(ep->release_dtd);
			ep->release_dtd = NULL;
		}
		free(mxc_udc.mxc_ep);
		mxc_udc.mxc_ep = NULL;
	}
	free(mxc_udc.release);
	mxc_udc.release = NULL;
}
/*
 * malloc an nocached memory
 * dmaaddr: phys address
 * size   : memory size
 * align  : alignment for this memroy
 * return : vir address(NULL when malloc failt)
*/
static void *malloc_dma_buffer(u32 *dmaaddr, void **release, int size, int align)
{
	void *virt;

	virt = memalign(align, size);
	if (*release)
		printf("%s: release bug\n", __func__);
	*release = virt;
#ifdef CONFIG_ARCH_MMU
	virt = ioremap_nocache(iomem_to_phys((unsigned long)virt), size);
#endif
	memset(virt, 0, size);
#ifdef CONFIG_ARCH_MMU
	*dmaaddr = (u32)iomem_to_phys((u32)virt);
#else
	*dmaaddr = (u32)virt;
#endif
	printf("virt addr %p, dma addr %x\n", virt, *dmaaddr);
	return virt;
}

int is_usb_disconnected()
{
	int ret = 0;

	ret = readl(USB_OTGSC) & OTGSC_B_SESSION_VALID ? 0 : 1;
	return ret;
}

static int mxc_init_usb_qh(void)
{
	int size;
	int i;
	mxc_ep_t *ep;

	ENTRY
	release_usb_space();
	memset(&mxc_udc, 0, sizeof(mxc_udc));
	mxc_udc.max_ep = (readl(USB_DCCPARAMS) & DCCPARAMS_DEN_MASK) * 2;
	DBG("udc max ep = %d\n", mxc_udc.max_ep);
	size = mxc_udc.max_ep * sizeof(struct ep_queue_head);
	mxc_udc.ep_qh = malloc_dma_buffer(&mxc_udc.qh_dma, &mxc_udc.release,
					     size, USB_MEM_ALIGN_BYTE);
	if (!mxc_udc.ep_qh) {
		printf("malloc ep qh dma buffer failure\n");
		return -1;
	}
	writel(mxc_udc.qh_dma & 0xfffff800, USB_ENDPOINTLISTADDR);

	size = mxc_udc.max_ep * sizeof(mxc_ep_t);
	DBG("init mxc ep\n");
	mxc_udc.mxc_ep = malloc(size);
	if (!mxc_udc.mxc_ep) {
		printf("malloc ep struct failure\n");
		return -1;
	}
	memset((void *)mxc_udc.mxc_ep, 0, size);
	ep  = mxc_udc.mxc_ep;
	for (i = 0; i < mxc_udc.max_ep; i++, ep++) {
		ep->epnum = i >> 1;
		ep->index = ep->done = 0;
		ep->dir = (i & 1) ? USB_SEND : USB_RECV;
		ep->ep_qh = &mxc_udc.ep_qh[i];
	}
	EXIT(0)
	return 0;
}

static int mxc_init_ep_dtd(u8 index)
{
	mxc_ep_t *ep;
	struct ep_queue_item *tqi;
	u32 dma;
	int i;
	int size;

	ENTRY
	if (index >= mxc_udc.max_ep)
		DBG("%s ep %d is not valid\n", __func__, index);

	ep = mxc_udc.mxc_ep + index;
	size = EP_TQ_ITEM_CNT * sizeof(struct ep_queue_item);
	tqi = malloc_dma_buffer(&dma, &ep->release_dtd, size, USB_MEM_ALIGN_BYTE);
	if (tqi == NULL) {
		printf("%s malloc tq item failure\n", __func__);
		return -1;
	}
	for (i = 0; i < EP_TQ_ITEM_CNT; i++) {
		ep->ep_dtd[i] = tqi;
		ep->ep_dtd[i]->item_dma = dma;
		tqi++;
		dma += sizeof(struct ep_queue_item);
	}
	return -1;
}
static void mxc_ep_qh_setup(u8 ep_num, u8 dir, u8 ep_type,
				 u32 max_pkt_len, u32 zlt, u8 mult)
{
	struct ep_queue_head *p_qh = mxc_udc.ep_qh + (2 * ep_num + dir);
	u32 tmp = 0;

	ENTRY
	tmp = max_pkt_len << 16;
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		tmp |= (1 << 15);
		break;
	case USB_ENDPOINT_XFER_ISOC:
		tmp |= (mult << 30);
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		break;
	default:
		DBG("error ep type is %d\n", ep_type);
		return;
	}
	if (zlt)
		tmp |= (1<<29);

	p_qh->config = tmp;
}

static void mxc_ep_setup(u8 ep_num, u8 dir, u8 ep_type)
{
	u32 epctrl = 0;

	ENTRY
	epctrl = readl(USB_ENDPTCTRL(ep_num));
	if (dir) {
		if (ep_num)
			epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		epctrl |= EPCTRL_TX_ENABLE;
		epctrl |= ((u32)(ep_type) << EPCTRL_TX_EP_TYPE_SHIFT);
	} else {
		if (ep_num)
			epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		epctrl |= EPCTRL_RX_ENABLE;
		epctrl |= ((u32)(ep_type) << EPCTRL_RX_EP_TYPE_SHIFT);
	}
	writel(epctrl, USB_ENDPTCTRL(ep_num));
}

static void mxc_tqi_init_page(struct ep_queue_item *tqi)
{
	tqi->page0 = tqi->page_dma;
	tqi->page1 = tqi->page0 + 0x1000;
	tqi->page2 = tqi->page1 + 0x1000;
	tqi->page3 = tqi->page2 + 0x1000;
	tqi->page4 = tqi->page3 + 0x1000;
}

#define ROUND_UP(a, b) ((((a) - 1) | ((b) - 1)) + 1)

int malloc_payload(mxc_ep_t *ep)
{
	int i;
	struct ep_queue_item *tqi;
	unsigned char *p;
	unsigned page_dma;
	int max = ROUND_UP(ep->max_pkt_size, USB_MEM_ALIGN_BYTE);

	p = malloc_dma_buffer(&page_dma, &ep->release_payload,
			max * EP_TQ_ITEM_CNT, USB_MEM_ALIGN_BYTE);
	if (!p) {
		printf("malloc ep's dtd bufer failure\n");
		return -1;
	}
	for (i = 0; i < EP_TQ_ITEM_CNT; i++) {
		tqi = ep->ep_dtd[i];
		tqi->page_vir = p;
		tqi->page_dma = page_dma;
		mxc_tqi_init_page(tqi);
		p += max;
		page_dma += max;
	}
	return 0;
}

static int mxc_malloc_ep0_ptr(mxc_ep_t *ep)
{
	ep->max_pkt_size = USB_MAX_CTRL_PAYLOAD;
	return malloc_payload(ep);
}

static void ep0_setup(void)
{
	mxc_ep_qh_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			    USB_MAX_CTRL_PAYLOAD, 0, 0);
	mxc_ep_qh_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
			    USB_MAX_CTRL_PAYLOAD, 0, 0);
	mxc_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
	mxc_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);
	mxc_init_ep_dtd(0 * 2 + USB_RECV);
	mxc_init_ep_dtd(0 * 2 + USB_SEND);
	mxc_malloc_ep0_ptr(mxc_udc.mxc_ep + (USB_RECV));
	mxc_malloc_ep0_ptr(mxc_udc.mxc_ep + (USB_SEND));
}


static int mxc_tqi_is_busy(struct ep_queue_item *tqi)
{
	/* bit 7 is set by software when send, clear by controller
	   when finish */
	return tqi->info & (1 << 7);
}

static int mxc_ep_xfer_is_working(mxc_ep_t *ep, u32 in)
{
	/* in: means device -> host */
	u32 bitmask = 1 << (ep->epnum + in * 16);
	u32 temp, prime, tstat;

	prime = (bitmask & readl(USB_ENDPTPRIME));
	if (prime)
		return 1;
	do {
		temp = readl(USB_USBCMD);
		writel(temp|USB_CMD_ATDTW, USB_USBCMD);
		tstat = readl(USB_ENDPTSTAT) & bitmask;
	} while (!(readl(USB_USBCMD) & USB_CMD_ATDTW));
	writel(temp & (~USB_CMD_ATDTW), USB_USBCMD);

	if (tstat)
		return 1;
	return 0;
}

static void mxc_update_qh(mxc_ep_t *ep, struct ep_queue_item *tqi, u32 in)
{
	/* in: means device -> host */
	struct ep_queue_head *qh = ep->ep_qh;
	u32 bitmask = 1 << (ep->epnum + in * 16);

	DBG("%s, line %d, epnum=%d, in=%d\n", __func__,
		__LINE__, ep->epnum, in);
	qh->next_queue_item = tqi->item_dma;
	qh->info = 0;
	writel(bitmask, USB_ENDPTPRIME);
}

static void _dump_buf(unsigned char *buf, u32 len)
{
#ifdef DEBUG
	int i;
	for (i = 0; i < len; i++)
		printf("%x ", buf[i]);
	printf("\n");
#endif
}

static void mxc_udc_dtd_update(mxc_ep_t *ep, u8 *data, u32 len)
{
	struct ep_queue_item *tqi, *head, *last;
	int max;
	int send = 0;
	int in = data ? 1 : 0;	/* an in endpoint will transit data */

	head = last = NULL;
	DBG("epnum = %d,  in = %d\n", ep->epnum, in);
	max = ROUND_UP(ep->max_pkt_size, USB_MEM_ALIGN_BYTE);
	do {
		int index = ep->index;
		tqi = ep->ep_dtd[index];
		DBG("%s, ep=%p, done=%d index = %d, tqi = %p\n", __func__, ep, ep->done, index, tqi);
		index = inc_index(index);
		if ((index == ep->done) || mxc_tqi_is_busy(tqi)) {
			printf("!!!BUG tqi=%p info=%x index=%x done=%x last=%p len=%x\n", tqi, tqi->info, index, ep->done, last, len);
			if (!last)
				return;
			break;
		}
		mxc_tqi_init_page(tqi);
		DBG("%s, line = %d, len = %d\n", __func__, __LINE__, len);
		ep->index = index;
		send = MIN(len, max);
		tqi->reserved[1] = (unsigned)data;
		if (data) {
			memcpy(tqi->page_vir, (void *)data, send);
			_dump_buf(tqi->page_vir, send);
			data += send;
		} else {
			tqi->reserved[0] = send;
		}
		if (!head)
			last = head = tqi;
		else {
			last->next_item_ptr = tqi->item_dma;
			last->next_item_vir = tqi;
			last = tqi;
		}
		/* we set IOS for every dtd */
		tqi->info = ((send << 16) | (1 << 15) | (1 << 7));
		len -= send;
	} while (len);

	last->next_item_ptr = 0x1; /* end */
	if (ep->tail) {
		ep->tail->next_item_ptr = head->item_dma;
		ep->tail->next_item_vir = head;
		if (mxc_ep_xfer_is_working(ep, in)) {
			DBG("ep is working\n");
			goto out;
		}
	}
	/* release previous unused setup */
	for (;;) {
		tqi = ep->ep_dtd[ep->done];
		if (tqi == head)
			break;
		if (!mxc_tqi_is_busy(tqi))
			break;	/* finished, but not processed yet */
		tqi->page_vir[80] = 0;
		/* This buffer will be orphaned if we don't mark as not busy */
		printf("Dropping bytes, info=%x data=0x%x '%s' epnum=%d, dir=%d, max_pkt_size=0x%x\n", tqi->info,
				tqi->reserved[1], tqi->page_vir, ep->epnum, ep->dir, ep->max_pkt_size);
		tqi->info &= ~(1 << 7);
		break;
	}
	mxc_update_qh(ep, head, in);
out:
	ep->tail = last;
}

static void mxc_udc_queue_update(u8 epnum, u8 *data, u32 len)
{
	int dir = data ? USB_SEND : USB_RECV;	/* USB_SEND(in endpoint) == 1*/
	mxc_ep_t *ep = mxc_udc.mxc_ep + (epnum * 2 + dir);

	mxc_udc_dtd_update(ep, data, len);
}

static void mxc_ep0_stall(void)
{
	u32 temp;

	temp = readl(USB_ENDPTCTRL(0));
	temp |= EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL;
	writel(temp, USB_ENDPTCTRL(0));
}

static void mxc_usb_run(void)
{
	unsigned int temp = 0;

	/* Enable DR irq reg */
	temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
		| USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
		| USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

	writel(temp, USB_USBINTR);

	/* Set controller to Run */
	temp = readl(USB_USBCMD);
	temp |= USB_CMD_RUN_STOP;
	writel(temp, USB_USBCMD);
}

static void mxc_usb_stop(void)
{
	unsigned int temp = 0;

	writel(temp, USB_USBINTR);

	/* disable pullup */
	/* Set controller to Stop */
	temp = readl(USB_USBCMD);
	temp &= ~USB_CMD_RUN_STOP;
	writel(temp, USB_USBCMD);
}

static void usb_phy_init(void)
{
	u32 temp;

	/* select 24M clk */
	temp = readl(USB_PHY1_CTRL);
	temp &= ~3;
	temp |= 1;
	writel(temp, USB_PHY1_CTRL);
	/* Config PHY interface */
	temp = readl(USB_PORTSC1);
	temp &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
	temp |= PORTSCX_PTW_16BIT;
	writel(temp, USB_PORTSC1);
	DBG("Config PHY  END\n");
}

static void usb_set_mode_device(void)
{
	u32 temp;

	/* Set controller to stop */
	temp = readl(USB_USBCMD);
	temp &= ~USB_CMD_RUN_STOP;
	writel(temp, USB_USBCMD);

	while (readl(USB_USBCMD) & USB_CMD_RUN_STOP)
		;
	/* Do core reset */
	temp = readl(USB_USBCMD);
	temp |= USB_CMD_CTRL_RESET;
	writel(temp, USB_USBCMD);
	while (readl(USB_USBCMD) & USB_CMD_CTRL_RESET)
		;
	DBG("DOORE RESET END\n");

#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
	reset_usb_phy1();
#endif
	DBG("init core to device mode\n");
	temp = readl(USB_USBMODE);
	temp &= ~USB_MODE_CTRL_MODE_MASK;	/* clear mode bits */
	temp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	temp |= USB_MODE_SETUP_LOCK_OFF;
	writel(temp, USB_USBMODE);
	DBG("init core to device mode end\n");
}

static void usb_init_eps(void)
{
	u32 temp;

	DBG("Flush begin\n");
	temp = readl(USB_ENDPTNAKEN);
	temp |= 0x10001;	/* clear mode bits */
	writel(temp, USB_ENDPTNAKEN);
	writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
	writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
	writel(0xffffffff, USB_ENDPTFLUSH);
	DBG("FLUSH END\n");
}

static void usb_udc_init(void)
{
	DBG("\n************************\n");
	DBG("         usb init start\n");
	DBG("\n************************\n");

	usb_phy_init();
	usb_set_mode_device();
	mxc_init_usb_qh();
	usb_init_eps();
	ep0_setup();
	usb_inited = 1;
}

void usb_shutdown(void)
{
	mxc_usb_stop();
	mdelay(2);
	release_usb_space();
}

static void ch9getstatus(u8 request_type, u16 value, u16 index, u16 length)
{
	u16 tmp;

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		tmp = 1 << 0; /* self powerd */
		tmp |= 0 << 1; /* not remote wakeup able */
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		tmp = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		tmp = 0;
	}
	mxc_udc_queue_update(0, (u8 *)&tmp, 2);
}
static void mxc_udc_read_setup_pkt(struct usb_device_request *s)
{
	u32 temp;

	temp = readl(USB_ENDPTSETUPSTAT);
	writel(temp | 1, USB_ENDPTSETUPSTAT);
	DBG("setup stat %x\n", temp);
	do {
		temp = readl(USB_USBCMD);
		temp |= USB_CMD_SUTW;
		writel(temp, USB_USBCMD);
		memcpy((void *)s,
			(void *)mxc_udc.mxc_ep[0].ep_qh->setup_data, 8);
	} while (!(readl(USB_USBCMD) & USB_CMD_SUTW));

	DBG("handle_setup s.type=%x req=%x len=%x\n",
		s->bmRequestType, s->bRequest, s->wLength);
	temp = readl(USB_USBCMD);
	temp &= ~USB_CMD_SUTW;
	writel(temp, USB_USBCMD);
}

static void mxc_udc_recv_setup(void)
{
	u8 tmp;
	struct usb_device_request *s = &mxc_udc.ep0_urb->device_request;
	int in = (s->bmRequestType & USB_DIR_IN);

	ENTRY
	mxc_udc_read_setup_pkt(s);
	if (ep0_recv_setup(mxc_udc.ep0_urb)) {
		mxc_ep0_stall();
		return;
	}
	switch (s->bRequest) {
	case USB_REQ_GET_STATUS:
		if ((s->bmRequestType & (USB_DIR_IN | USB_TYPE_MASK)) !=
					 (USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		ch9getstatus(s->bmRequestType, s->wValue,
				s->wIndex, s->wLength);
		goto status_phase;
	case USB_REQ_SET_ADDRESS:
		if (s->bmRequestType != (USB_DIR_OUT |
			    USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;
		mxc_udc.setaddr = 1;
		mxc_udc_queue_update(0, &tmp, 0);	/* status */
		usbd_device_event_irq(udc_device, DEVICE_ADDRESS_ASSIGNED, 0);
		return;
	case USB_REQ_SET_CONFIGURATION:
		usbd_device_event_irq(udc_device, DEVICE_CONFIGURED, 0);
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
	{
		int field = s->bmRequestType & (USB_TYPE_MASK | USB_RECIP_MASK);
		if ((field == (USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)) ||
		    (field == (USB_TYPE_STANDARD | USB_RECIP_DEVICE))) {
			in = 0;
			goto status_phase;
		}
		break;
	}
	default:
		break;
	}
	if (s->wLength) {
		mxc_udc_queue_update(0, in ? mxc_udc.ep0_urb->buffer : NULL,
				mxc_udc.ep0_urb->actual_length);
		mxc_udc.ep0_urb->actual_length = 0;
	} else {
		in = 0;
	}
	/*
	 * status phase is in opposite direction
	 * send/receive zero-length packet response for success
	 */
status_phase:
	mxc_udc_queue_update(0, in ? NULL : &tmp, 0);
}

static int _mxc_ep_recv_data(struct usb_endpoint_instance *epi, struct ep_queue_item *tqi)
{
	struct urb *urb = epi->rcv_urb;
	u32 len = 0;

	if (urb) {
		u8 *data = urb->buffer + urb->actual_length;
		int urb_rem = urb->buffer_length - urb->actual_length;
		int remain_len = (tqi->info >> 16) & (0xefff);
		len = tqi->reserved[0] - remain_len;
		if (tqi->info & ((1 << 5) | (1 << 3)))
			printf("%s: error: remain_len=%x send=%x info=%x\n", __func__, remain_len, tqi->reserved[0], tqi->info);
		if (len > urb_rem) {
			if (!urb_rem)
				return -1;
			printf("err: len=%x, urb_rem=%x\n", len, urb_rem);
			len = urb_rem;
		}
		DBG("recv len %d-%d-%d\n", len, tqi->reserved[0], remain_len);
		memcpy(data, tqi->page_vir, len);
	}
	return len;
}

static void mxc_udc_ep_recv(mxc_ep_t *ep, unsigned max_length)
{
	struct ep_queue_item *tqi;

	DBG("%s: ep=%p, done=%x index=%x\n", __func__, ep, ep->done, ep->index);
	while (1) {
		int nbytes;
		if (ep->done == ep->index)
			break;
		tqi = ep->ep_dtd[ep->done];
		if (mxc_tqi_is_busy(tqi))
			break;
		nbytes = _mxc_ep_recv_data(ep->epi, tqi);
		if (nbytes < 0)
			break;
		if (nbytes) {
			/*
			 * Zero max_length on short packet reception
			 */
			if (nbytes & (ep->max_pkt_size - 1))
				max_length = 0;
			usbd_rcv_complete(ep->epi, nbytes, 0);
		}
		ep->done = inc_index(ep->done);
	}

	if (max_length) {
		int used = (ep->index - ep->done) & (EP_TQ_ITEM_CNT - 1);
		int free = EP_TQ_ITEM_CNT - 1 - used;

		if (0) if (max_length < 0xC000)
			printf("%s: free=0x%x, used=0x%x, max_length=0x%x max_pkt_size=0x%x\n",
				__func__, free, used, max_length, ep->max_pkt_size);
		if (free >= (EP_TQ_ITEM_CNT / 2)) {
			unsigned max = ROUND_UP(ep->max_pkt_size, USB_MEM_ALIGN_BYTE);
			unsigned urb_len = ep->epi->rcv_urb->actual_length;

			used *= max;
			if (max_length > urb_len)
				max_length -= urb_len;
			else
				max_length = ep->max_pkt_size;
			if (max_length > used) {
				max_length -= used;
				free *= max;
				if (free > max_length) {
					free = ROUND_UP(max_length, ep->max_pkt_size);
				}
				mxc_udc_dtd_update(ep, NULL, free);
				if (0) if (max_length < 0xC000) {
					printf("%s: free=0x%x, used=0x%x, max_length=0x%x\n",
						__func__, free, used, max_length);
					used = (ep->index - ep->done) & (EP_TQ_ITEM_CNT - 1);
					printf("%s: used=0x%x urb_len=0x%x\n", __func__, used, urb_len);
				}
			}
		}
	}
}

void usb_rcv_urb(struct usb_endpoint_instance *epi, unsigned max_length)
{
	int epnum = epi->endpoint_address & USB_ENDPOINT_NUMBER_MASK;
	mxc_ep_t *ep = mxc_udc.mxc_ep + (epnum * 2 + USB_RECV);

	mxc_udc_ep_recv(ep, max_length);
}

static void mxc_udc_ep_tx(mxc_ep_t *ep)
{
	struct ep_queue_item *tqi;
	int complete = 0;

	DBG("%s: ep=%p, done=%x index=%x\n", __func__, ep, ep->done, ep->index);
	while (1) {
		if (ep->done == ep->index) {
			if (complete) {
				DBG("%s: complete\n", __func__);
				usbd_tx_complete(ep->epi);
			}
			break;
		}
		tqi = ep->ep_dtd[ep->done];
		if (mxc_tqi_is_busy(tqi))
			break;
		complete = 1;
		ep->done = inc_index(ep->done);
	}
}

unsigned get_mask(unsigned max_ep)
{
	unsigned mask1 = (1 << ((max_ep + 1) / 2)) - 1;
	unsigned mask2 = (1 << (max_ep / 2)) - 1;
	return mask1 | (mask2 << 16);
}

static void mxc_udc_handle_xfer_complete(void)
{
	u32 bitpos = readl(USB_ENDPTCOMPLETE);
	mxc_ep_t *ep = mxc_udc.mxc_ep;

	writel(bitpos, USB_ENDPTCOMPLETE);
	DBG("%s: bitpos=%x\n", __func__, bitpos);

	if (mxc_udc.setaddr) {
		writel(udc_device->address << 25, USB_DEVICEADDR);
		mxc_udc.setaddr = 0;
	}

	bitpos &= get_mask(mxc_udc.max_ep);
	DBG("%s: masked bitpos=%x\n", __func__, bitpos);
	while (bitpos) {
		if (bitpos & 1)
			mxc_udc_ep_recv(ep, 0);
		ep++;

		if (bitpos & 0x10000) {
			bitpos &= ~0x10000;
			mxc_udc_ep_tx(ep);
		}
		bitpos >>= 1;
		ep++;
	}
}
static void usb_dev_hand_usbint(void)
{
	if (readl(USB_ENDPTSETUPSTAT)) {
		DBG("recv one setup packet\n");
		mxc_udc_recv_setup();
	}
	if (readl(USB_ENDPTCOMPLETE)) {
		DBG("Dtd complete irq\n");
		mxc_udc_handle_xfer_complete();
	}
}

static void usb_dev_hand_reset(void)
{
	u32 temp;

	temp = readl(USB_DEVICEADDR);
	temp &= ~0xfe000000;
	writel(temp, USB_DEVICEADDR);
	writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
	writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
	while (readl(USB_ENDPTPRIME))
		;
	writel(0xffffffff, USB_ENDPTFLUSH);
	DBG("reset-PORTSC=%x\n", readl(USB_PORTSC1));
	usbd_device_event_irq(udc_device, DEVICE_RESET, 0);
}

/*
 * Returns 2 for highspeed
 */
int usb_get_port_speed(void)
{
	int speed;

	while (readl(USB_PORTSC1) & PORTSCX_PORT_RESET)
		;
	speed = readl(USB_PORTSC1) & PORTSCX_PORT_SPEED_MASK;
	switch (speed) {
	case PORTSCX_PORT_SPEED_HIGH:
		speed = 2;
		break;
	case PORTSCX_PORT_SPEED_FULL:
		speed = 1;
		break;
	case PORTSCX_PORT_SPEED_LOW:
		speed = 0;
		break;
	default:
		speed = -EINVAL;
		break;
	}
	return speed;
}
void usb_dev_hand_pci(void)
{
	usb_highspeed = usb_get_port_speed();
	DBG("portspeed=%d\n", usb_highspeed);
}

void usb_dev_hand_suspend(void)
{
}
static int ll;
void mxc_irq_poll(void)
{
	unsigned irq_src = readl(USB_USBSTS) & readl(USB_USBINTR);

	writel(irq_src, USB_USBSTS);

	if (irq_src == 0)
		return;

	EXIT(irq_src)
	if (irq_src & USB_STS_INT) {
		ll++;
		DBG("USB_INT\n");
		usb_dev_hand_usbint();
	}
	if (irq_src & USB_STS_RESET) {
		printf("USB_RESET\n");
		usb_dev_hand_reset();
	}
	if (irq_src & USB_STS_PORT_CHANGE)
		usb_dev_hand_pci();
	if (irq_src & USB_STS_SUSPEND)
		printf("USB_SUSPEND\n");
	if (irq_src & USB_STS_ERR)
		printf("USB_ERR, %x\n", irq_src);
}

void mxc_udc_wait_cable_insert(void)
{
	u32 temp;
	int cable_connect = 1;

	do {
		udelay(50);

		temp = readl(USB_OTGSC);
		if (temp & (OTGSC_B_SESSION_VALID)) {
			printf("USB Mini b cable Connected!\n");
			break;
		} else if (cable_connect == 1) {
			printf("wait usb cable into the connector!\n");
			cable_connect = 0;
		}
	} while (1);
}

int udc_disable_over_current(void)
{
	u32 temp;

	temp = readl(USB_OTG_CTRL);
	temp |= UCTRL_OVER_CUR_POL;
	writel(temp, USB_OTG_CTRL);
	return 0;
}

/*
 * mxc_udc_init function
 */
int mxc_udc_init(void)
{
	set_usboh3_clk();
	set_usb_phy1_clk();
	enable_usboh3_clk(1);
#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
	udc_disable_over_current();
#endif
	enable_usb_phy1_clk(1);
	usb_udc_init();

	return 0;
}

void mxc_udc_poll(void)
{
	mxc_irq_poll();
}

/*
 * Functions for gadget APIs
 */
int udc_init(void)
{
	mxc_udc_init();
	return 0;
}

void udc_setup_ep(struct usb_device_instance *device, u32 index,
		    struct usb_endpoint_instance *epi)
{
	u8 dir, epnum, zlt, mult;
	u8 ep_type;
	u32 max_pkt_size;
	int ep_addr;
	mxc_ep_t *ep;

	ENTRY
	if (epi) {
		zlt = 1;
		mult = 0;
		ep_addr = epi->endpoint_address;
		epnum = ep_addr & USB_ENDPOINT_NUMBER_MASK;
		DBG("setup ep %d\n", epnum);
		if ((ep_addr & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
			dir = USB_SEND;
			ep_type = epi->tx_attributes;
			max_pkt_size = epi->tx_packetSize;
		} else {
			dir = USB_RECV;
			ep_type = epi->rcv_attributes;
			max_pkt_size = epi->rcv_packetSize;
		}
		if (ep_type == USB_ENDPOINT_XFER_ISOC) {
			mult = (u32)(1 + ((max_pkt_size >> 11) & 0x03));
			max_pkt_size = max_pkt_size & 0x7ff;
			DBG("mult = %d\n", mult);
		}
		ep = mxc_udc.mxc_ep + (epnum * 2 + dir);
		ep->epi = epi;
		if (epnum) {
			mxc_ep_qh_setup(epnum, dir, ep_type,
					    max_pkt_size, zlt, mult);
			mxc_ep_setup(epnum, dir, ep_type);
			mxc_init_ep_dtd(epnum * 2 + dir);

			/* malloc endpoint's dtd's data buffer*/
			ep->max_pkt_size = max_pkt_size;
			malloc_payload(ep);
			if (dir == USB_RECV)
				mxc_udc_ep_recv(ep, max_pkt_size);
		}
	}
}


int udc_endpoint_write(struct usb_endpoint_instance *epi)
{
	struct urb *urb;
	int ep_num;
	u8 *data;
	int n;

	if (epi->tx_ready.next == &epi->tx_ready)
		return 0;

	urb = p2surround(struct urb, link, epi->tx_ready.next);

	ep_num = epi->endpoint_address & USB_ENDPOINT_NUMBER_MASK;
	data = (u8 *)urb->buffer + epi->sent;
	n = urb->actual_length - epi->sent;

	DBG("data=%p, n=%x buffer=%p sent=%x actual_length=%x\n", data, n,
			urb->buffer, epi->sent, urb->actual_length);
	mxc_udc_queue_update(ep_num, data, n);
	epi->last = n;
	return 0;
}

void udc_enable(struct usb_device_instance *device)
{
	udc_device = device;
	mxc_udc.ep0_urb = usbd_alloc_urb(udc_device, udc_device->bus->endpoint_array);
}

void udc_startup_events(struct usb_device_instance *device)
{
	usbd_device_event_irq(device, DEVICE_INIT, 0);
	usbd_device_event_irq(device, DEVICE_CREATE, 0);
	udc_enable(device);
}

void udc_irq(void)
{
	mxc_irq_poll();
}

void udc_connect(void)
{
	mxc_usb_run();
//	mxc_udc_wait_cable_insert();
}

void udc_disconnect(void)
{
	/* imx6 will hang if access usb register without init oh3
	 * clock, so not access it if not init. */
	if (usb_inited)
		mxc_usb_stop();
}

void udc_set_nak(int epid)
{
}

void udc_unset_nak(int epid)
{
}

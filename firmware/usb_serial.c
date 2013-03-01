/* USB Serial Example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_serial.html
 * Copyright (c) 2008,2010,2011 PJRC.COM, LLC
 *
 * This is NOT the original version of the code.
 * If you are interrested in it, visit the above mentioned web site.
 *
 * Copyright (c) 2013 Jan Seiffert, Frederic Endner-Duehr, Dirk Raufer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * The main difference between this code and the orignal is the cut down
 * on interrupts-off-time.
 * This reduces delay and jitter when other, more important interrupt driven
 * HW needs to be serviced.
 *
 * This comes at a price, notably:
 * 1) There is no implicit flushing
 *    YOU need to explicitly flush the TX buffer, it will not be done
 *    with some timeout.
 * 2) The code can't be called from interrupt context.
 *    If you want to print (we are talking about a serial link to some host
 *    here) during an interrupt handler, you have other problems...
 * 3) The code is more racy against removal/detach during operation.
 *    This is kind of a red hering, since because you switched off interrupts
 *    and such so a state change is not delivered to the code, doesn't mean
 *    the USB-Core isn't in a wedged/unhappy state while you still try to
 *    push bytes into it
 */


// Version 1.0: Initial Release
// Version 1.1: support Teensy++
// Version 1.2: fixed usb_serial_available
// Version 1.3: added transmit bandwidth test
// Version 1.4: added usb_serial_write
// Version 1.5: add support for Teensy 2.0
// Version 1.6: fix zero length packet bug
// Version 1.7: fix usb_serial_set_control

#include <stdbool.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "usb_serial.h"
#include "helper.h"

/**************************************************************************
 *
 *  Configurable Options
 *
 **************************************************************************/

/*
 * You can change these to give your code its own name.  On Windows,
 * these are only used before an INF file (driver install) is loaded.
 */
#define STR_MANUFACTURER        L"Your Name"
#define STR_PRODUCT             L"USB Serial"

/*
 * All USB serial devices are supposed to have a serial number
 * (according to Microsoft).  On windows, a new COM port is created
 * for every unique serial/vendor/product number combination.  If
 * you program 2 identical boards with 2 different serial numbers
 * and they are assigned COM7 and COM8, each will always get the
 * same COM port number because Windows remembers serial numbers.
 *
 * On Mac OS-X, a device file is created automatically which
 * incorperates the serial number, eg, /dev/cu-usbmodem12341
 *
 * Linux by default ignores the serial number, and creates device
 * files named /dev/ttyACM0, /dev/ttyACM1... in the order connected.
 * Udev rules (in /etc/udev/rules.d) can define persistent device
 * names linked to this serial number, as well as permissions, owner
 * and group settings.
 */
#define STR_SERIAL_NUMBER       L"12345"

/*
 * Mac OS-X and Linux automatically load the correct drivers.  On
 * Windows, even though the driver is supplied by Microsoft, an
 * INF file is needed to load the driver.  These numbers need to
 * match the INF file.
 */
#define VENDOR_ID               0x16C0
#define PRODUCT_ID              0x047A

/*
 * When you write data, it goes into a USB endpoint buffer, which
 * is transmitted to the PC when it becomes full, or after a timeout
 * with no more writes.  Even if you write in exactly packet-size
 * increments, this timeout is used to send a "zero length packet"
 * that tells the PC no more data is expected and it should pass
 * any buffered data to the application that may be waiting.  If
 * you want data sent immediately, call usb_serial_flush_output().
 */
#define TRANSMIT_FLUSH_TIMEOUT  5   /* in milliseconds */

/*
 * If the PC is connected but not "listening", this is the length
 * of time before usb_serial_getchar() returns with an error.  This
 * is roughly equivilant to a real UART simply transmitting the
 * bits on a wire where nobody is listening, except you get an error
 * code which you can ignore for serial-like discard of data, or
 * use to know your data wasn't sent.
 */
#define TRANSMIT_TIMEOUT        25   /* in milliseconds */

/*
 * USB devices are supposed to implment a halt feature, which is
 * rarely (if ever) used.  If you comment this line out, the halt
 * code will be removed, saving 116 bytes of space (gcc 4.3.0).
 * This is not strictly USB compliant, but works with all major
 * operating systems.
 */
#define SUPPORT_ENDPOINT_HALT


/**************************************************************************
 *
 *  Internal defines
 *
 **************************************************************************/

#ifndef EPINT_D0
# define EPINT_D0 0
#endif

#define EP_TYPE_CONTROL             0x00
#define EP_TYPE_BULK_IN             0x81
#define EP_TYPE_BULK_OUT            0x80
#define EP_TYPE_INTERRUPT_IN        0xC1
#define EP_TYPE_INTERRUPT_OUT       0xC0
#define EP_TYPE_ISOCHRONOUS_IN      0x41
#define EP_TYPE_ISOCHRONOUS_OUT     0x40
#define EP_SINGLE_BUFFER            0x02
#define EP_DOUBLE_BUFFER            0x06
#define EP_SIZE(s)  ((s) == 64 ? 0x30 : \
                    ((s) == 32 ? 0x20 : \
                    ((s) == 16 ? 0x10 : \
                                 0x00)))

#define MAX_ENDPOINT                4

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

#if defined(__AVR_AT90USB162__)
# define HW_CONFIG()
# define PLL_CONFIG() (PLLCSR = (_BV(PLLE)|_BV(PLLP0)))
# define USB_CONFIG() (USBCON = _BV(USBE))
# define USB_FREEZE() (USBCON = (_BV(USBE)|_BV(FRZCLK)))
#elif defined(__AVR_ATmega32U4__)
# define HW_CONFIG()  (UHWCON = 0x01)
# define PLL_CONFIG() (PLLCSR = 0x12)
# define USB_CONFIG() (USBCON = (_BV(USBE)|_BV(OTGPADE)))
# define USB_FREEZE() (USBCON = (_BV(USBE)|_BV(FRZCLK)))
#elif defined(__AVR_AT90USB646__)
# define HW_CONFIG()  (UHWCON = 0x81)
# define PLL_CONFIG() (PLLCSR = 0x1A)
# define USB_CONFIG() (USBCON = (_BV(USBE)|_BV(OTGPADE)))
# define USB_FREEZE() (USBCON = (_BV(USBE)|_BV(FRZCLK)))
#elif defined(__AVR_AT90USB1286__)
# define HW_CONFIG()  (UHWCON = 0x81)
# define PLL_CONFIG() (PLLCSR = 0x16)
# define USB_CONFIG() (USBCON = (_BV(USBE)|_BV(OTGPADE)))
# define USB_FREEZE() (USBCON = (_BV(USBE)|_BV(FRZCLK)))
#endif

/* standard control endpoint request types */
#define GET_STATUS                  0
#define CLEAR_FEATURE               1
#define SET_FEATURE                 3
#define SET_ADDRESS                 5
#define GET_DESCRIPTOR              6
#define GET_CONFIGURATION           8
#define SET_CONFIGURATION           9
#define GET_INTERFACE               10
#define SET_INTERFACE               11
/* HID (human interface device) */
#define HID_GET_REPORT              1
#define HID_GET_PROTOCOL            3
#define HID_SET_REPORT              9
#define HID_SET_IDLE                10
#define HID_SET_PROTOCOL            11
/* CDC (communication class device) */
#define CDC_SET_LINE_CODING         0x20
#define CDC_GET_LINE_CODING         0x21
#define CDC_SET_CONTROL_LINE_STATE  0x22


/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/

/*
 * These buffer sizes are best for most applications, but perhaps if you
 * want more buffering on some endpoint at the expense of others, this
 * is where you can make such changes.  The AT90USB162 has only 176 bytes
 * of DPRAM (USB buffers) and only endpoints 3 & 4 can double buffer.
 */
#define ENDPOINT0_SIZE      16
#define CDC_ACM_ENDPOINT    2
#define CDC_RX_ENDPOINT     3
#define CDC_TX_ENDPOINT     4
#if defined(__AVR_AT90USB162__)
# define CDC_ACM_SIZE        16
# define CDC_ACM_BUFFER      EP_SINGLE_BUFFER
# define CDC_RX_SIZE         32
# define CDC_RX_BUFFER       EP_DOUBLE_BUFFER
# define CDC_TX_SIZE         32
# define CDC_TX_BUFFER       EP_DOUBLE_BUFFER
#else
# define CDC_ACM_SIZE        16
# define CDC_ACM_BUFFER      EP_SINGLE_BUFFER
# define CDC_RX_SIZE         64
# define CDC_RX_BUFFER       EP_DOUBLE_BUFFER
# define CDC_TX_SIZE         64
# define CDC_TX_BUFFER       EP_DOUBLE_BUFFER
#endif

static const uint8_t PROGMEM endpoint_config_table[] = {
	0,
	1, EP_TYPE_INTERRUPT_IN,  EP_SIZE(CDC_ACM_SIZE) | CDC_ACM_BUFFER,
	1, EP_TYPE_BULK_OUT,      EP_SIZE(CDC_RX_SIZE)  | CDC_RX_BUFFER,
	1, EP_TYPE_BULK_IN,       EP_SIZE(CDC_TX_SIZE)  | CDC_TX_BUFFER
};


/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

/*
 * Descriptors are the data that your computer reads when it auto-detects
 * this USB device (called "enumeration" in USB lingo).  The most commonly
 * changed items are editable at the top of this file.  Changing things
 * in here should only be done by those who've read chapter 9 of the USB
 * spec and relevant portions of any USB class specifications!
 */
static const uint8_t PROGMEM device_descriptor[] = {
	18,                                 /* bLength */
	1,                                  /* bDescriptorType */
	0x00, 0x02,                         /* bcdUSB */
	2,                                  /* bDeviceClass */
	0,                                  /* bDeviceSubClass */
	0,                                  /* bDeviceProtocol */
	ENDPOINT0_SIZE,                     /* bMaxPacketSize0 */
	LSB(VENDOR_ID), MSB(VENDOR_ID),     /* idVendor */
	LSB(PRODUCT_ID), MSB(PRODUCT_ID),   /* idProduct */
	0x00, 0x01,                         /* bcdDevice */
	1,                                  /* iManufacturer */
	2,                                  /* iProduct */
	3,                                  /* iSerialNumber */
	1                                   /* bNumConfigurations */
};

#define CONFIG1_DESC_SIZE (9+9+5+5+4+5+7+9+7+7)
static const uint8_t PROGMEM config1_descriptor[CONFIG1_DESC_SIZE] = {
	/* configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10 */
	9,                                  /* bLength */
	2,                                  /* bDescriptorType */
	LSB(CONFIG1_DESC_SIZE),             /* wTotalLength */
	MSB(CONFIG1_DESC_SIZE),
	2,                                  /* bNumInterfaces */
	1,                                  /* bConfigurationValue */
	0,                                  /* iConfiguration */
	0xC0,                               /* bmAttributes */
	50,                                 /* bMaxPower */
	/* interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12 */
	9,                                  /* bLength */
	4,                                  /* bDescriptorType */
	0,                                  /* bInterfaceNumber */
	0,                                  /* bAlternateSetting */
	1,                                  /* bNumEndpoints */
	0x02,                               /* bInterfaceClass */
	0x02,                               /* bInterfaceSubClass */
	0x01,                               /* bInterfaceProtocol */
	0,                                  /* iInterface */
	/* CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26 */
	5,                                  /* bFunctionLength */
	0x24,                               /* bDescriptorType */
	0x00,                               /* bDescriptorSubtype */
	0x10, 0x01,                         /* bcdCDC */
	/* Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27 */
	5,                                  /* bFunctionLength */
	0x24,                               /* bDescriptorType */
	0x01,                               /* bDescriptorSubtype */
	0x01,                               /* bmCapabilities */
	1,                                  /* bDataInterface */
	/* Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28 */
	4,                                  /* bFunctionLength */
	0x24,                               /* bDescriptorType */
	0x02,                               /* bDescriptorSubtype */
	0x06,                               /* bmCapabilities */
	/* Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33 */
	5,                                  /* bFunctionLength */
	0x24,                               /* bDescriptorType */
	0x06,                               /* bDescriptorSubtype */
	0,                                  /* bMasterInterface */
	1,                                  /* bSlaveInterface0 */
	/* endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13 */
	7,                                  /* bLength */
	5,                                  /* bDescriptorType */
	CDC_ACM_ENDPOINT | 0x80,            /* bEndpointAddress */
	0x03,                               /* bmAttributes (0x03=intr) */
	CDC_ACM_SIZE, 0,                    /* wMaxPacketSize */
	64,                                 /* bInterval */
	/* interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12 */
	9,                                  /* bLength */
	4,                                  /* bDescriptorType */
	1,                                  /* bInterfaceNumber */
	0,                                  /* bAlternateSetting */
	2,                                  /* bNumEndpoints */
	0x0A,                               /* bInterfaceClass */
	0x00,                               /* bInterfaceSubClass */
	0x00,                               /* bInterfaceProtocol */
	0,                                  /* iInterface */
	/* endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13 */
	7,                                  /* bLength */
	5,                                  /* bDescriptorType */
	CDC_RX_ENDPOINT,                    /* bEndpointAddress */
	0x02,                               /* bmAttributes (0x02=bulk) */
	CDC_RX_SIZE, 0,                     /* wMaxPacketSize */
	0,                                  /* bInterval */
	/* endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13 */
	7,                                  /* bLength */
	5,                                  /* bDescriptorType */
	CDC_TX_ENDPOINT | 0x80,             /* bEndpointAddress */
	0x02,                               /* bmAttributes (0x02=bulk) */
	CDC_TX_SIZE, 0,                     /* wMaxPacketSize */
	0                                   /* bInterval */
};

/*
 * If you're desperate for a little extra code memory, these strings
 * can be completely removed if iManufacturer, iProduct, iSerialNumber
 * in the device desciptor are changed to zeros.
 */
struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	int16_t wString[];
};
static const struct usb_string_descriptor_struct PROGMEM string0 = {
	4,
	3,
	{0x0409}
};
static const struct usb_string_descriptor_struct PROGMEM string1 = {
	sizeof(STR_MANUFACTURER),
	3,
	STR_MANUFACTURER
};
static const struct usb_string_descriptor_struct PROGMEM string2 = {
	sizeof(STR_PRODUCT),
	3,
	STR_PRODUCT
};
static const struct usb_string_descriptor_struct PROGMEM string3 = {
	sizeof(STR_SERIAL_NUMBER),
	3,
	STR_SERIAL_NUMBER
};

/*
 * This table defines which descriptor data is sent for each specific
 * request from the host (in wValue and wIndex).
 */
static const struct descriptor_list_struct {
	uint16_t        wValue;
	uint16_t        wIndex;
	const uint8_t   *addr;
	uint8_t         length;
} PROGMEM descriptor_list[] = {
	{0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
	{0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
	{0x0300, 0x0000, (const uint8_t *)&string0, 4},
	{0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
	{0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)},
	{0x0303, 0x0409, (const uint8_t *)&string3, sizeof(STR_SERIAL_NUMBER)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor_list_struct))


/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/

/* zero when we are not configured, non-zero when enumerated */
static volatile uint8_t usb_configuration=0;

/*
 * the time remaining before we transmit any partially full
 * packet, or send a zero length packet.
 */
static volatile uint8_t transmit_flush_timer=0;
static uint8_t transmit_previous_timeout=0;

/*
 * serial port settings (baud rate, control signals, etc) set
 * by the PC.  These are ignored, but kept in RAM.
 */
static uint8_t cdc_line_coding[7]={0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x08};
volatile uint8_t sx_cdc_line_rtsdtr=0;


/**************************************************************************
 *
 *  Public Functions - these are the API intended for the user
 *
 **************************************************************************/

/* initialize USB serial */
void usb_init(void)
{
	HW_CONFIG();
	USB_FREEZE();                       /* enable USB */
	PLL_CONFIG();                       /* config PLL, 16 MHz xtal */
	while (!(PLLCSR & _BV(PLOCK)))
		/* nop */;                       /* wait for PLL lock */
	USB_CONFIG();                       /* start USB clock */
	UDCON = 0;                          /* enable attach resistor */
	usb_configuration = 0;
	sx_cdc_line_rtsdtr = 0;
	UDIEN = _BV(EORSTE)|_BV(SOFE);
	sei();
}

/*
 * return 0 if the USB is not configured, or the configuration
 * number selected by the HOST
 */
uint8_t usb_configured(void)
{
	return usb_configuration;
}

/* get the next character, or -1 if nothing received */
int16_t usb_serial_getchar(void)
{
	uint8_t c, intr_state;

	/*
	 * interrupts are disabled so these functions can be
	 * used from the main program or interrupt context,
	 * even both in the same program!
	 */
	intr_state = SREG;
	cli();
	if (!usb_configuration) {
		SREG = intr_state;
		return -1;
	}
	UENUM = CDC_RX_ENDPOINT;
retry:
	c = UEINTX;
	if (!(c & _BV(RWAL))) {
		/* no data in buffer */
		if (c & _BV(RXOUTI)) {
			UEINTX = 0x6B;
			goto retry;
		}
		SREG = intr_state;
		return -1;
	}
	/* take one byte out of the buffer */
	c = UEDATX;
	/* if buffer completely used, release it */
	if (!(UEINTX & _BV(RWAL)))
		UEINTX = 0x6B;
	SREG = intr_state;
	return c;
}

/* number of bytes available in the receive buffer */
uint8_t usb_serial_available(void)
{
	uint8_t n=0, i, intr_state;

	intr_state = SREG;
	cli();
	if (usb_configuration) {
		UENUM = CDC_RX_ENDPOINT;
		n = UEBCLX;
		if (!n) {
			i = UEINTX;
			if (i & _BV(RXOUTI) && !(i & _BV(RWAL)))
				UEINTX = 0x6B;
		}
	}
	SREG = intr_state;
	return n;
}

/* discard any buffered input */
void usb_serial_flush_input(void)
{
	uint8_t intr_state;

	if (usb_configuration) {
		intr_state = SREG;
		cli();
		UENUM = CDC_RX_ENDPOINT;
		while ((UEINTX & _BV(RWAL))) {
			UEINTX = 0x6B;
		}
		SREG = intr_state;
	}
}

/* transmit a character.  0 returned on success, -1 on error */
int8_t GCC_ATTR_OPTIMIZE("O3") usb_serial_putchar(uint8_t c)
{
	uint8_t timeout;
	bool o_flush_timer;

	/* if we're not online (enumerated and configured), error */
	if (unlikely(!usb_configuration))
		return -1;

	/* reset flush timer to prevent interrupt from flushing while we write */
	cli();
	o_flush_timer = !!transmit_flush_timer;
	transmit_flush_timer = 0;
	sei();

	UENUM = CDC_TX_ENDPOINT;
	/* if we gave up due to timeout before, don't wait again */
	if (unlikely(transmit_previous_timeout)) {
		if (!(UEINTX & _BV(RWAL)))
			goto OUT_ERR;
		transmit_previous_timeout = 0;
	}
	/* wait for the FIFO to be ready to accept data */
	timeout = UDFNUML + TRANSMIT_TIMEOUT;
	while (1) {
		/* are we ready to transmit? */
		if (likely(UEINTX & _BV(RWAL)))
			break;
		/*
		 * have we waited too long?  This happens if the user
		 * is not running an application that is listening
		 */
		if (unlikely(UDFNUML == timeout)) {
			transmit_previous_timeout = 1;
			goto OUT_ERR;
		}
		/* has the USB gone offline? */
		if (unlikely(!usb_configuration))
			return -1;
		/* get ready to try checking again */
		if (unlikely(o_flush_timer)) {
			transmit_flush_timer = 1;
			o_flush_timer = false;
		}
	}
	/* actually write the byte into the FIFO */
	UEDATX = c;
	/* if this completed a packet, transmit it now! */
	if (!(UEINTX & _BV(RWAL)))
		UEINTX = 0x3A;

	/* set new flush timeout */
	transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	return 0;

OUT_ERR:
	transmit_flush_timer = 1;
	return 1;
}


/*
 * transmit a character, but do not wait if the buffer is full,
 *  0 returned on success, -1 on buffer full or error
 */
int8_t usb_serial_putchar_nowait(uint8_t c)
{
	uint8_t intr_state;

	if (!usb_configuration)
		return -1;
	intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	if (!(UEINTX & _BV(RWAL))) {
		/* buffer is full */
		SREG = intr_state;
		return -1;
	}
	/* actually write the byte into the FIFO */
	UEDATX = c;
	/* if this completed a packet, transmit it now! */
	if (!(UEINTX & _BV(RWAL)))
		UEINTX = 0x3A;
	transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	SREG = intr_state;
	return 0;
}

/*
 * transmit a buffer.
 *  0 returned on success, -1 on error
 * This function is optimized for speed!  Each call takes approx 6.1 us overhead
 * plus 0.25 us per byte.  12 Mbit/sec USB has 8.67 us per-packet overhead and
 * takes 0.67 us per byte.  If called with 64 byte packet-size blocks, this function
 * can transmit at full USB speed using 43% CPU time.  The maximum theoretical speed
 * is 19 packets per USB frame, or 1216 kbytes/sec.  However, bulk endpoints have the
 * lowest priority, so any other USB devices will likely reduce the speed.  Speed
 * can also be limited by how quickly the PC-based software reads data, as the host
 * controller in the PC will not allocate bandwitdh without a pending read request.
 * (thanks to Victor Suarez for testing and feedback and initial code)
 */
int8_t usb_serial_write(const uint8_t *buffer, uint16_t size)
{
	uint8_t intr_state;

	/* if we're not online (enumerated and configured), error */
	if (!usb_configuration)
		return -1;
	/*
	 * interrupts are disabled so these functions can be
	 * used from the main program or interrupt context,
	 * even both in the same program!
	 */
	intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	/* if we gave up due to timeout before, don't wait again */
	if (transmit_previous_timeout) {
		if (!(UEINTX & _BV(RWAL))) {
			SREG = intr_state;
			return -1;
		}
		transmit_previous_timeout = 0;
	}
	/* each iteration of this loop transmits a packet */
	while (size) {
		uint8_t write_size;
		{
		/* wait for the FIFO to be ready to accept data */
		uint8_t timeout = UDFNUML + TRANSMIT_TIMEOUT;
		while (1) {
			/* are we ready to transmit? */
			if (UEINTX & _BV(RWAL))
				break;
			SREG = intr_state;
			/*
			 * have we waited too long?  This happens if the user
			 * is not running an application that is listening
			 */
			if (UDFNUML == timeout) {
				transmit_previous_timeout = 1;
				return -1;
			}
			/* has the USB gone offline? */
			if (!usb_configuration)
				return -1;
			/* get ready to try checking again */
			intr_state = SREG;
			cli();
			UENUM = CDC_TX_ENDPOINT;
		}
		}

		/* compute how many bytes will fit into the next packet */
		write_size = CDC_TX_SIZE - UEBCLX;
#if (CDC_TX_SIZE > 64)
		if (write_size > 64)
			write_size = 64;
#endif
		if (write_size > size)
			write_size = size;
		size -= write_size;

#ifdef MAD_ASM_SKILLZ
		/*
		 * unfortunatly the compiler is not to smart to unravel this switch
		 *
		 * 6c4:       0c 94 1c 08     jmp     0x1038  ; 0x1038 <__tablejump2__>
		 * 6c8:       f8 01           movw    r30, r16
		 * 6ca:       81 91           ld      r24, Z+
		 * 6cc:       8f 01           movw    r16, r30
		 * 6ce:       80 93 f1 00     sts     0x00F1, r24
		 * 6d2:       f8 01           movw    r30, r16
		 * 6d4:       81 91           ld      r24, Z+
		 * 6d6:       8f 01           movw    r16, r30
		 * ......
		 *
		 * 1) the worst part is, he desparatly tries to move the buffer
		 *    pointer around and around and around, for every byte,
		 *    only to not store it in Z, two extra instruction per byte...
		 * 2) besides that and the wasted flash, he hardcodes the
		 *    UEDATX location into instructions again and again.
		 *    What's nice on a once off access, is bad on repeated
		 *    access, wasting more flash. Most compiler suck at the
		 *    decision when better to materialize a constant in a reg
		 *    and when to sink it into the instruction
		 * 3) and to make matters worse he uses the __tablejump2__ helper,
		 *    which never heard of ijmp (instead 2*push (2* 3 clock) +
		 *    ret (5 clock) + other)
		 * 4) and to round it of he uses a jumptable (more flash) with
		 *    program memory access (slow). Yeah, only few compiler can do
		 *    calculated jumps.
		 *
		 * so fix this by manual intervention, not pretty, but since this
		 * is to copy many bytes, give it the love it deserves.
		 */
		uint16_t t;
		asm volatile (
			"subi %A[temp], pm_lo8(-(1f))\n\t"
			"sbci %B[temp], pm_hi8(-(1f))\n\t"
			"ijmp\n\t"
			"1:\n\t"
# define cpy_one() "ld %A[temp], %a[buffer]+\n\tst %a[uedatx], %A[temp]\n\t"
# define cpy_two() cpy_one() cpy_one()
# define cpy_four() cpy_two() cpy_two()
# define cpy_eight() cpy_four() cpy_four()
# if (CDC_TX_SIZE == 64)
			cpy_eight()
			cpy_eight()
			cpy_eight()
			cpy_eight()
# endif
# if (CDC_TX_SIZE >= 32)
			cpy_eight()
			cpy_eight()
# endif
# if (CDC_TX_SIZE >= 16)
			cpy_eight()
# endif
			cpy_eight()
			/*  0 */
			: /* %0 */ [buffer] "=e" (buffer),
			  /* %1 */ [temp]   "=z" (t)
			: /* %2 */ [uedatx] "e" (&UEDATX),
			  /*    */ "0" (buffer),
			  /*    */ "1" ((CDC_TX_SIZE - write_size)*2)
		);
# undef cpy_eight
# undef cpy_four
# undef cpy_two
# undef cpy_one
#else
# define cpy_one(x) case x: UEDATX = *buffer++;
# define cpy_two(x) cpy_one(x) cpy_one(x+1)
# define cpy_four(x) cpy_two(x) cpy_two(x+2)
# define cpy_eight(x) cpy_four(x) cpy_four(x+4)
		/* write the packet */
		switch (write_size) {
# if (CDC_TX_SIZE == 64)
			cpy_eight(57)
			cpy_eight(49)
			cpy_eight(41)
			cpy_eight(33)
# endif
# if (CDC_TX_SIZE >= 32)
			cpy_eight(25)
			cpy_eight(17)
# endif
# if (CDC_TX_SIZE >= 16)
			cpy_eight(9)
# endif
			cpy_eight(1)
			case  0: break;
		}
# undef cpy_eight
# undef cpy_four
# undef cpy_two
# undef cpy_one
#endif
		/* if this completed a packet, transmit it now! */
		if (!(UEINTX & _BV(RWAL)))
			UEINTX = 0x3A;
		transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
		SREG = intr_state;
	}
	return 0;
}

int8_t usb_serial_write_str(const char *str)
{
	uint8_t intr_state;

	/* if we're not online (enumerated and configured), error */
	if (!usb_configuration)
		return -1;
	/*
	 * interrupts are disabled so these functions can be
	 * used from the main program or interrupt context,
	 * even both in the same program!
	 */
	intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	/* if we gave up due to timeout before, don't wait again */
	if (transmit_previous_timeout) {
		if (!(UEINTX & _BV(RWAL))) {
			SREG = intr_state;
			return -1;
		}
		transmit_previous_timeout = 0;
	}
	/* each iteration of this loop transmits a packet */
	while (*str) {
		uint8_t write_size;
		char c;
		{
		/* wait for the FIFO to be ready to accept data */
		uint8_t timeout = UDFNUML + TRANSMIT_TIMEOUT;
		while (1) {
			/* are we ready to transmit? */
			if (UEINTX & _BV(RWAL))
				break;
			SREG = intr_state;
			/*
			 * have we waited too long?  This happens if the user
			 * is not running an application that is listening
			 */
			if (UDFNUML == timeout) {
				transmit_previous_timeout = 1;
				return -1;
			}
			/* has the USB gone offline? */
			if (!usb_configuration)
				return -1;
			/* get ready to try checking again */
			intr_state = SREG;
			cli();
			UENUM = CDC_TX_ENDPOINT;
		}
		}

		/* compute how many bytes will fit into the next packet */
		write_size = CDC_TX_SIZE - UEBCLX;

		/* write the packet */
		for (c = *str++; c && write_size; c = *str++, write_size--)
			UEDATX = c;
		/* if this completed a packet, transmit it now! */
		if (!(UEINTX & _BV(RWAL)))
			UEINTX = 0x3A;
		transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
		SREG = intr_state;
	}
	return 0;
}

int8_t usb_serial_write_str_PGM(PGM_P str)
{
	uint8_t intr_state;

	/* if we're not online (enumerated and configured), error */
	if (!usb_configuration)
		return -1;
	/*
	 * interrupts are disabled so these functions can be
	 * used from the main program or interrupt context,
	 * even both in the same program!
	 */
	intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	/* if we gave up due to timeout before, don't wait again */
	if (transmit_previous_timeout) {
		if (!(UEINTX & _BV(RWAL))) {
			SREG = intr_state;
			return -1;
		}
		transmit_previous_timeout = 0;
	}
	/* each iteration of this loop transmits a packet */
	while (pgm_read_byte(str)) {
		uint8_t write_size;
		char c;
		{
		/* wait for the FIFO to be ready to accept data */
		uint8_t timeout = UDFNUML + TRANSMIT_TIMEOUT;
		while (1) {
			/* are we ready to transmit? */
			if (UEINTX & _BV(RWAL))
				break;
			SREG = intr_state;
			/*
			 * have we waited too long?  This happens if the user
			 * is not running an application that is listening
			 */
			if (UDFNUML == timeout) {
				transmit_previous_timeout = 1;
				return -1;
			}
			/* has the USB gone offline? */
			if (!usb_configuration)
				return -1;
			/* get ready to try checking again */
			intr_state = SREG;
			cli();
			UENUM = CDC_TX_ENDPOINT;
		}
		}

		/* compute how many bytes will fit into the next packet */
		write_size = CDC_TX_SIZE - UEBCLX;

		/* write the packet */
		for (c = pgm_read_byte(str++); c && write_size; c = pgm_read_byte(str++), write_size--)
			UEDATX = c;
		/* if this completed a packet, transmit it now! */
		if (!(UEINTX & _BV(RWAL)))
			UEINTX = 0x3A;
		transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
		SREG = intr_state;
	}
	return 0;
}

/*
 * immediately transmit any buffered output.
 * This doesn't actually transmit the data - that is impossible!
 * USB devices only transmit when the host allows, so the best
 * we can do is release the FIFO buffer for when the host wants it
 */
void usb_serial_flush_output(void)
{
	cli();
	if (transmit_flush_timer) {
		transmit_flush_timer = 0;
		sei();
		UENUM = CDC_TX_ENDPOINT;
		UEINTX = 0x3A;
	}
	else
		sei();
}

/*
 * write the control signals, DCD, DSR, RI, etc
 * There is no CTS signal.  If software on the host has transmitted
 * data to you but you haven't been calling the getchar function,
 * it remains buffered (either here or on the host) and can not be
 * lost because you weren't listening at the right time, like it
 * would in real serial communication.
 */
int8_t usb_serial_set_control(uint8_t signals)
{
	uint8_t intr_state;

	intr_state = SREG;
	cli();
	if (!usb_configuration) {
		/* we're not enumerated/configured */
		SREG = intr_state;
		return -1;
	}

	UENUM = CDC_ACM_ENDPOINT;
	if (!(UEINTX & _BV(RWAL))) {
		/*
		 * unable to write
		 * TODO; should this try to abort the previously
		 * buffered message??
		 */
		SREG = intr_state;
		return -1;
	}
	UEDATX = 0xA1;
	UEDATX = 0x20;
	UEDATX = 0;
	UEDATX = 0;
	UEDATX = 0; /* 0 seems to work nicely.  what if this is 1?? */
	UEDATX = 0;
	UEDATX = 1;
	UEDATX = 0;
	UEDATX = signals;
	UEINTX = 0x3A;
	SREG = intr_state;
	return 0;
}



/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/


/*
 * USB Device Interrupt - handle all device-level events
 * the transmit buffer flushing is triggered by the start of frame
 */
ISR(USB_GEN_vect, GCC_ATTR_OPTIMIZE("O3"))
{
	uint8_t intbits;

	intbits = UDINT;
	UDINT = 0;
	if (unlikely(intbits & _BV(EORSTI)))
	{
		/*
		 * This is racy with the other routines, even if they switch off
		 * interrupts.
		 * Wenn the USB-Controller is "suddenly" in reset state (or is
		 * otherwise incapacitated), just because the interrupt is not
		 * delivered (yet), does not mean we can still use the controller.
		 * So even with interrupts off, if the USB controller goes away
		 * in the middle of the function, we can do nothing about it.
		 */
		uint8_t o_uenum = UENUM;
		UENUM = 0;
		UECONX = 1;
		UECFG0X = EP_TYPE_CONTROL;
		UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
		UEIENX = _BV(RXSTPE);
		usb_configuration = 0;
		sx_cdc_line_rtsdtr = 0;
		UENUM = o_uenum;
	}
	if (intbits & (1<<SOFI)) {
		if (likely(usb_configuration)) {
			uint8_t t = transmit_flush_timer;
			if (unlikely(t)) {
				transmit_flush_timer = --t;
				if (!t) {
					uint8_t o_uenum = UENUM;
					UENUM = CDC_TX_ENDPOINT;
					UEINTX = 0x3A;
					UENUM = o_uenum;
				}
			}
		}
	}
}


/* Misc functions to wait for ready and send/receive packets */
static inline void usb_wait_in_ready(void)
{
	while (!(UEINTX & _BV(TXINI))) ;
}
static inline void usb_send_in(void)
{
	UEINTX = ~_BV(TXINI);
}
static inline void usb_wait_receive_out(void)
{
	while (!(UEINTX & _BV(RXOUTI))) ;
}
static inline void usb_ack_out(void)
{
	UEINTX = ~_BV(RXOUTI);
}


/*
 * USB Endpoint Interrupt - endpoint 0 is handled here.  The
 * other endpoints are manipulated by the user-callable
 * functions, and the start-of-frame interrupt.
 */
static noinline void ep0_com_isr(void)
{
	uint8_t intbits;
	const uint8_t *list;
	const uint8_t *cfg;
	uint8_t i, n, len, en;
	uint8_t *p;
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint16_t desc_val;
	const uint8_t *desc_addr;
	uint8_t desc_length;

	UENUM = 0;
	intbits = UEINTX;
	if (intbits & _BV(RXSTPI)) {
		bmRequestType = UEDATX;
		bRequest = UEDATX;
		wValue = UEDATX;
		wValue |= (UEDATX << 8);
		wIndex = UEDATX;
		wIndex |= (UEDATX << 8);
		wLength = UEDATX;
		wLength |= (UEDATX << 8);
		UEINTX = ~(_BV(RXSTPI) | _BV(RXOUTI) | _BV(TXINI));
		if (bRequest == GET_DESCRIPTOR) {
			list = (const uint8_t *)descriptor_list;
			for (i = 0; ; i++) {
				if (i >= NUM_DESC_LIST) {
					UECONX = _BV(STALLRQ)|_BV(EPEN);  /* stall */
					return;
				}
				desc_val = pgm_read_word(list);
				if (desc_val != wValue) {
					list += sizeof(struct descriptor_list_struct);
					continue;
				}
				list += 2;
				desc_val = pgm_read_word(list);
				if (desc_val != wIndex) {
					list += sizeof(struct descriptor_list_struct)-2;
					continue;
				}
				list += 2;
				desc_addr = (const uint8_t *)pgm_read_word(list);
				list += 2;
				desc_length = pgm_read_byte(list);
				break;
			}
			len = (wLength < 256) ? wLength : 255;
			if (len > desc_length)
				len = desc_length;
			do {
				/* wait for host ready for IN packet */
				do {
					i = UEINTX;
				} while (!(i & (_BV(TXINI)|_BV(RXOUTI))));
				if (i & _BV(RXOUTI))
					return;    /* abort */
				/* send IN packet */
				n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
				for (i = n; i; i--) {
					UEDATX = pgm_read_byte(desc_addr++);
				}
				len -= n;
				usb_send_in();
			} while (len || n == ENDPOINT0_SIZE);
			return;
		}
		if (bRequest == SET_ADDRESS) {
			usb_send_in();
			usb_wait_in_ready();
			UDADDR = wValue | _BV(ADDEN);
			return;
		}
		if (bRequest == SET_CONFIGURATION && bmRequestType == 0) {
			usb_configuration = wValue;
			sx_cdc_line_rtsdtr = 0;
			transmit_flush_timer = 0;
			usb_send_in();
			cfg = endpoint_config_table;
			for (i = 1; i < 5; i++) {
				UENUM = i;
				en = pgm_read_byte(cfg++);
				UECONX = en;
				if (en) {
					UECFG0X = pgm_read_byte(cfg++);
					UECFG1X = pgm_read_byte(cfg++);
				}
			}
			UERST = 0x1E;
			UERST = 0;
			return;
		}
		if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80) {
			usb_wait_in_ready();
			UEDATX = usb_configuration;
			usb_send_in();
			return;
		}
		if (bRequest == CDC_GET_LINE_CODING && bmRequestType == 0xA1) {
			usb_wait_in_ready();
			p = cdc_line_coding;
			for (i = 0; i < 7; i++) {
				UEDATX = *p++;
			}
			usb_send_in();
			return;
		}
		if (bRequest == CDC_SET_LINE_CODING && bmRequestType == 0x21) {
			usb_wait_receive_out();
			p = cdc_line_coding;
			for (i = 0; i < 7; i++) {
				*p++ = UEDATX;
			}
			usb_ack_out();
			usb_send_in();
			return;
		}
		if (bRequest == CDC_SET_CONTROL_LINE_STATE && bmRequestType == 0x21) {
			sx_cdc_line_rtsdtr = wValue;
			usb_wait_in_ready();
			usb_send_in();
			return;
		}
		if (bRequest == GET_STATUS) {
			usb_wait_in_ready();
			i = 0;
#ifdef SUPPORT_ENDPOINT_HALT
			if (bmRequestType == 0x82) {
				UENUM = wIndex;
				if (UECONX & _BV(STALLRQ))
					i = 1;
				UENUM = 0;
			}
#endif
			UEDATX = i;
			UEDATX = 0;
			usb_send_in();
			return;
		}
#ifdef SUPPORT_ENDPOINT_HALT
		if ((bRequest == CLEAR_FEATURE || bRequest == SET_FEATURE)
				&& bmRequestType == 0x02 && wValue == 0) {
			i = wIndex & 0x7F;
			if (i >= 1 && i <= MAX_ENDPOINT) {
				usb_send_in();
				UENUM = i;
				if (bRequest == SET_FEATURE) {
					UECONX = _BV(STALLRQ)|_BV(EPEN);
				} else {
					UECONX = _BV(STALLRQC)|_BV(RSTDT)|_BV(EPEN);
					UERST = 1 << i;
					UERST = 0;
				}
				return;
			}
		}
#endif
	}
	UECONX = _BV(STALLRQ) | _BV(EPEN);  /* stall */
}

/* if any other endpoint gives an interrupt, shut them all off! */
static noinline void disable_other_endpoints(void)
{
	// TODO: shut them off
	/* for the time beeing, do what old code does, stall EP0 */
	UENUM = 0;
	UECONX = _BV(STALLRQ) | _BV(EPEN);  /* stall */
}

ISR(USB_COM_vect)
{
	uint8_t o_uenum, ep_source;

	/* get original UENUM value */
	o_uenum = UENUM;
	/* get interrupt source mask */
	ep_source = UEINT;

	if(ep_source & _BV(EPINT_D0))
		ep0_com_isr();
	if(ep_source & ~_BV(EPINT_D0))
		disable_other_endpoints();

	/* set UENUM to original value */
	UENUM = o_uenum;
}

/* EOF */

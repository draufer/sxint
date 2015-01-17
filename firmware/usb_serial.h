#ifndef usb_serial_h__
#define usb_serial_h__

#include <stdint.h>
#include <avr/pgmspace.h>

/* setup */
void usb_init(void);                            /* initialize everything */
uint8_t usb_configured(void);                   /* is the USB port configured */

/* receiving data */
int16_t usb_serial_getchar(void);               /* receive a character (-1 if timeout/error) */
uint8_t usb_serial_available(void);             /* number of bytes in receive buffer */
void usb_serial_flush_input(void);              /* discard any buffered input */

/* transmitting data */
int8_t usb_serial_putchar(uint8_t c);           /* transmit a character */
int8_t usb_serial_putchar_nowait(uint8_t c);    /* transmit a character, do not wait */
int8_t usb_serial_write(const uint8_t *buffer, uint16_t size); /* transmit a buffer */
int8_t usb_serial_write_str(const char *str);   /* transmit a NUL terminated string */
int8_t usb_serial_write_str_PGM(PGM_P str);     /* transmit a NUL terminated string from program memory */
void usb_serial_flush_output(void);             /* immediately transmit any buffered output */

/* serial parameters */
extern volatile uint8_t sx_cdc_line_rtsdtr;
#define usb_serial_get_control() sx_cdc_line_rtsdtr /* get the RTS and DTR signal state */
int8_t usb_serial_set_control(uint8_t signals); /* set DSR, DCD, RI, etc */

/* draufer */
void usb_enable_interrupts(void);
void usb_disable_interrupts(void);

/* constants corresponding to the various serial parameters */
#define USB_SERIAL_DTR              0x01
#define USB_SERIAL_RTS              0x02
#define USB_SERIAL_1_STOP           0
#define USB_SERIAL_1_5_STOP         1
#define USB_SERIAL_2_STOP           2
#define USB_SERIAL_PARITY_NONE      0
#define USB_SERIAL_PARITY_ODD       1
#define USB_SERIAL_PARITY_EVEN      2
#define USB_SERIAL_PARITY_MARK      3
#define USB_SERIAL_PARITY_SPACE     4
#define USB_SERIAL_DCD              0x01
#define USB_SERIAL_DSR              0x02
#define USB_SERIAL_BREAK            0x04
#define USB_SERIAL_RI               0x08
#define USB_SERIAL_FRAME_ERR        0x10
#define USB_SERIAL_PARITY_ERR       0x20
#define USB_SERIAL_OVERRUN_ERR      0x40

/*
 * This file does not include the HID debug functions, so these empty
 * macros replace them with nothing, so users can compile code that
 * has calls to these functions.
 */
#define usb_debug_putchar(c)
#define usb_debug_flush_output()


#endif

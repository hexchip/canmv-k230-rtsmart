#ifndef DEV_USBH_SERIAL_H
#define DEV_USBH_SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <usbh_core.h>
#include <usb_cdc.h>

typedef struct dev_usbh_serial_ops {
    char* (*get_dev_path)(void *usbh_serial);
    int (*set_line_coding)(void *usbh_serial, struct cdc_line_coding *line_coding);
    int (*get_line_coding)(void *usbh_serial, struct cdc_line_coding *line_coding);
    int (*set_line_state)(void *usbh_serial, bool dtr, bool rts);
    void (*set_bulk_in_transfer_buffer)(void *usbh_serial, uint8_t *buffer, uint32_t buflen);
    void (*set_bulk_out_transfer_buffer)(void *usbh_serial, uint8_t *buffer, uint32_t buflen);
    int (*bulk_in_transfer)(void *usbh_serial, uint32_t timeout);
    int (*bulk_out_transfer)(void *usbh_serial, uint32_t timeout);
    int (*async_bulk_in_transfer)(void *usbh_serial, usbh_complete_callback_t complete, void *arg);
    int (*async_bulk_out_transfer)(void *usbh_serial, usbh_complete_callback_t complete, void *arg);
    int (*cancel_bulk_in_transfer)(void *usbh_serial);
    int (*cancel_bulk_out_transfer)(void *usbh_serial);
} dev_usbh_serial_ops_t;

typedef struct usbh_serial_dev usbh_serial_dev_t;

usbh_serial_dev_t* dev_usbh_serial_create(void *usbh_serial, dev_usbh_serial_ops_t *ops, size_t ring_buffer_size);

void dev_usbh_serial_destroy(usbh_serial_dev_t* dev);

#endif // DEV_USBH_SERIAL_H
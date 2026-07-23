/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

/* ================ USB common Configuration ================ */

#define CONFIG_USB_PRINTF(...) //printf(__VA_ARGS__)

#define usb_malloc(size) malloc(size)
#define usb_free(ptr)    free(ptr)

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_NONE
#endif

/* Enable print with color */
#define CONFIG_USB_PRINTF_COLOR_ENABLE

/* data align size when use dma */
#ifndef CONFIG_USB_ALIGN_SIZE
#define CONFIG_USB_ALIGN_SIZE 4
#endif

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* ================= USB Device Stack Configuration ================ */

/* Ep0 max transfer buffer, specially for receiving data from ep0 out */
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 80


/* ================ USB Device Port Configuration ================*/
#include "py32f0xx.h"

#define USBD_IRQn       USB_IRQn

#define USBD_IRQHandler USB_IRQHandler

typedef struct
{
    uint8_t *buf;
    const uint32_t size;
    volatile uint32_t *write_pointer;
} cdc_acm_rx_buf_t;

void cdc_acm_init(cdc_acm_rx_buf_t rx_buf);
void cdc_acm_data_send_with_dtr(const uint8_t *buf, uint32_t size);
void cdc_acm_data_send_with_dtr_async(const uint8_t *buf, uint32_t size);

#endif

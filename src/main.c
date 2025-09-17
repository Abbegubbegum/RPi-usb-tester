/**
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <bsp/board_api.h>
#include <tusb.h>

#include <pico/stdio.h>

#include "include.h"

static uint8_t vnd_rx_buf[512];
static uint8_t vnd_tx_buf[512];

int main(void)
{
    // Initialize TinyUSB stack
    board_init();
    tusb_init();

    // TinyUSB board init callback after init
    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

    // let pico sdk use the first cdc interface for std io
    stdio_init_all();

    // main run loop
    while (1)
    {
        // TinyUSB device task | must be called regularly
        tud_task();
    }

    // indicate no error
    return 0;
}

// callback when data is received on a CDC interface
void tud_cdc_rx_cb(uint8_t itf)
{
    // allocate buffer for the data in the stack
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    printf("RX CDC %d\n", itf);

    // read the available data
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
    buf[count] = 0; // null-terminate the string

    tud_cdc_n_write(itf, (uint8_t const *)"OK\r\n", 4);
    tud_cdc_n_write_flush(itf);
}

static uint8_t *rx_buffer[512];
static uint16_t rx_bufsize;

// VENDOR BULK: echo anything we receive
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    // printf("Received 0x%02x bytes of data\n", bufsize);

    memcpy(rx_buffer, buffer, bufsize);
    rx_bufsize = bufsize;

    // printf("Data: [");
    // for (int i = 0; i < bufsize; i++)
    // {
    //     printf("0x%02x, ", buffer[i]);
    // }
    // printf("]\n");

    tud_vendor_write(rx_buffer, rx_bufsize);
    tud_vendor_flush();

#if CFG_TUD_VENDOR_RX_BUFSIZE > 0
    tud_vendor_read_flush();
#endif
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    printf("Sent 0x%02x bytes\n", sent_bytes);
}

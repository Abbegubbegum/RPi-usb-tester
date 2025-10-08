#include <stdlib.h>
#include <bsp/board_api.h>
#include <tusb.h>

#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>

#include <hardware/uart.h>
#include <hardware/irq.h>

#include "include.h"

static uint8_t vnd_buffer[1512];
static uint16_t vnd_buflen = 0;
static uint16_t expected_packet_size = 0;

static uint8_t usb_selected_port = 0;

void service_vendor();
void on_uart_rx();

static void init_gpio()
{
    // Initialize the GPIO pin for USB mux selection
    gpio_init(USB_MUX_SEL_PIN);
    gpio_set_dir(USB_MUX_SEL_PIN, GPIO_OUT);
    gpio_put(USB_MUX_SEL_PIN, 0);
}

static void init_uart()
{
    // Set up UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // Set up an interrupt on RX
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    uart_puts(UART_ID, "\nUART initialized\n");
}

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

    init_gpio();
    init_uart();
    // main run loop
    while (1)
    {
        // TinyUSB device task | must be called regularly
        tud_task();

        service_vendor();
    }

    // indicate no error
    return 0;
}

void usb_switch_to_port(uint8_t port)
{
    if (port > 1)
        return; // Invalid port

    tud_disconnect();
    sleep_ms(3); // Wait for host to notice disconnection

    gpio_put(USB_MUX_SEL_PIN, port);

    sleep_ms(5); // Wait for the mux to switch
    tud_connect();

    uart_puts(UART_ID, "Switched USB MUX to port ");
    uart_putc(UART_ID, '0' + port);
    uart_puts(UART_ID, "\n");
    usb_selected_port = port;
}

// callback when data is received on a CDC interface
void tud_cdc_rx_cb(uint8_t itf)
{
    // allocate buffer for the data in the stack
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    // read the available data
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
    buf[count] = 0; // null-terminate the string
    tud_cdc_n_write(itf, (uint8_t const *)"RX: ", 4);
    tud_cdc_n_write(itf, buf, count);
    tud_cdc_n_write(itf, (uint8_t const *)"\n", 1);
    tud_cdc_n_write_flush(itf);
}

// VENDOR BULK: echo anything we receive
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    // printf("Received 0x%02x bytes of data\n", bufsize);

    //     memcpy(vnd_buffer, buffer, bufsize);
    //     vnd_bufsize = bufsize;

    //     // printf("Data: [");
    //     // for (int i = 0; i < bufsize; i++)
    //     // {
    //     //     printf("0x%02x, ", buffer[i]);
    //     // }
    //     // printf("]\n");

    //     tud_vendor_write(vnd_buffer, vnd_bufsize);
    //     tud_vendor_flush();

    // #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
    //     tud_vendor_read_flush();
    // #endif
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    // printf("Sent 0x%04x bytes\n", sent_bytes);
}

// Protocol:
// 0x01 - GET_PORT
// 0x02 - GET_POWER
// 0x03 - SET_PORT <port>
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP)
        return true; // only handle setup stage

    char print1[32];
    snprintf(print1, sizeof(print1), "Vendor control request: 0x%02x", request->bRequest);
    char print2[32];
    snprintf(print2, sizeof(print2), "  wValue: 0x%04x\n", request->wValue);

    uart_puts(UART_ID, print1);
    uart_puts(UART_ID, print2);

    switch (request->bRequest)
    {
    case 0x01:
    {
        // GET_PORT
        uart_puts(UART_ID, "Reporting current port: ");
        uart_putc(UART_ID, '0' + usb_selected_port);
        uart_puts(UART_ID, "\n");

        bool result = tud_control_xfer(rhport, request, &usb_selected_port, 1);
        uart_puts(UART_ID, result ? "Sent port\n" : "Failed to send port\n");
        return result;
    }
    case 0x02:
    {
        // GET_POWER
        uint8_t power = 200; // mA
        return tud_control_xfer(rhport, request, &power, 1);
    }
    case 0x03:
    {
        // SET_PORT
        usb_switch_to_port((uint8_t)request->wValue);
        return tud_control_status(rhport, request); // send status response
    }
    default:
        // stall unknown request
        return false;
    }
    return false; // should not reach here
}

void service_vendor()
{
    while (tud_vendor_available())
    {
        // Read as much data as we can into the vnd_buffer
        uint32_t count = tud_vendor_read(vnd_buffer + vnd_buflen, sizeof(vnd_buffer) - vnd_buflen);
        vnd_buflen += count;

        // If we have not parsed a header yet and have enough bits in the buffer
        if (expected_packet_size == 0 && vnd_buflen >= 6)
        {
            // printf("Parsing header: ");
            // Parse little endian
            uint32_t seq = (uint32_t)vnd_buffer[0] | ((uint32_t)vnd_buffer[1] << 8) |
                           ((uint32_t)vnd_buffer[2] << 16) | ((uint32_t)vnd_buffer[3] << 24);
            uint16_t len = (uint16_t)vnd_buffer[4] | ((uint16_t)vnd_buffer[5] << 8);

            // printf("seq: %d, size: %d\n", seq, len);

            expected_packet_size = (uint32_t)len;
        }

        if (expected_packet_size && vnd_buflen >= expected_packet_size)
        {
            // printf("Received enough bytes, sending packet\n");
            tud_vendor_write(vnd_buffer, expected_packet_size);
            tud_vendor_flush();

            uint32_t remain = vnd_buflen - expected_packet_size;
            memmove(vnd_buffer, vnd_buffer + vnd_buflen, remain);
            vnd_buflen = remain;
            expected_packet_size = 0;
        }
    }
}

#define CMD_BUFFER_SIZE 32

static char cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_buflen = 0;

void process_command(const char *cmd)
{
    if (strcmp(cmd, "set 1") == 0)
    {
        usb_switch_to_port(1);
    }
    else if (strcmp(cmd, "set 0") == 0)
    {
        usb_switch_to_port(0);
    }
    else if (strcmp(cmd, "get") == 0)
    {
        char response[32];
        snprintf(response, sizeof(response), "USB MUX state: %d\n", usb_selected_port);
        uart_puts(UART_ID, response);
    }
    else if (strcmp(cmd, "reboot") == 0)
    {
        uart_puts(UART_ID, "Rebooting into bootloader...\n");
        sleep_ms(100); // Give time for message to be sent
        reset_usb_boot(0, 0);
    }
    else
    {
        uart_puts(UART_ID, "Unknown command\n");
    }
}

void on_uart_rx()
{
    if (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);
        // Echo back the received character
        uart_putc(UART_ID, ch);

        if (ch == '\r' || ch == '\n')
        {
            if (cmd_buflen == 0)
                return; // Ignore empty commands
            // End of command
            cmd_buffer[cmd_buflen] = 0; // Null-terminate the string
            cmd_buflen = 0;             // Reset buffer length for next command
            uart_puts(UART_ID, "\n");   // New line after command

            // Process command
            process_command(cmd_buffer);
        }
        else if (ch == 8 || ch == 127)
        {
            if (cmd_buflen > 0)
            {
                // Handle backspace/delete: remove last char and erase it on the terminal
                cmd_buflen--;
                // uart_putc(UART_ID, '\b');
                // uart_putc(UART_ID, ' ');
                // uart_putc(UART_ID, '\b');
            }
        }
        else if (cmd_buflen < CMD_BUFFER_SIZE - 1)
        {
            // Store character in command buffer
            cmd_buffer[cmd_buflen++] = ch;
        }
    }
}
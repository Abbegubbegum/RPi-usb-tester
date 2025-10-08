#include <stdlib.h>
#include <bsp/board_api.h>
#include <tusb.h>

#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>

#include <hardware/uart.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/watchdog.h>

#include "include.h"

static uint8_t g_vnd_buffer[1280];
static uint16_t g_vnd_buflen = 0;
static uint16_t g_vnd_expected_packet_size = 0;

static uint8_t g_active_port = 0;
static uint8_t g_port_map = 0;   // Bitmap of which ports have been connected to the host (VBUS present)
static uint8_t g_port_count = 0; // Number of enumerated ports
static bool g_mounted = false;

static bool g_pending_switch = false;
static bool g_switch_request_finished = true;
static uint8_t g_pending_port = 0;

static char g_cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t g_cmd_buflen = 0;

void service_vendor();
void on_uart_rx();
void scan_present_ports();
void usb_switch_to_port(uint8_t port);

void print_fmt(const char *fmt, ...)
{
    char buf[64];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        uart_puts(UART_ID, buf);
    }
}

static void init_gpio()
{
    // Initialize the GPIO pin for USB mux selection
    gpio_init(USB_MUX_SEL_PIN);
    gpio_set_dir(USB_MUX_SEL_PIN, GPIO_OUT);
    gpio_put(USB_MUX_SEL_PIN, 0);

    gpio_init(PORT_0_VBUS_SENSE_PIN);
    gpio_set_dir(PORT_0_VBUS_SENSE_PIN, GPIO_IN);
    gpio_disable_pulls(PORT_0_VBUS_SENSE_PIN);
    gpio_set_input_hysteresis_enabled(PORT_0_VBUS_SENSE_PIN, true);

    gpio_init(PORT_1_VBUS_SENSE_PIN);
    gpio_set_dir(PORT_1_VBUS_SENSE_PIN, GPIO_IN);
    gpio_disable_pulls(PORT_1_VBUS_SENSE_PIN);
    gpio_set_input_hysteresis_enabled(PORT_1_VBUS_SENSE_PIN, true);
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

    print_fmt("\nUART initialized\n");
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

    adc_init();

    init_gpio();
    init_uart();

    watchdog_enable(2000, 1); // 2 second timeout
    // main run loop

    scan_present_ports();
    while (1)
    {
        // TinyUSB device task | must be called regularly
        tud_task();

        service_vendor();

        if (g_pending_switch && g_switch_request_finished)
        {
            g_pending_switch = false;
            usb_switch_to_port(g_pending_port);
        }

        watchdog_update();
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

    print_fmt("Switched USB MUX to port %d\n", port);
    g_active_port = port;
}

uint16_t adc_mv(uint16_t adc_code)
{
    uint32_t mv = (uint32_t)VREF_MV * adc_code / ADC_MAX;
    return (uint16_t)((mv * DIV_RATIO_X100 + 50) / 100); // rounding
}

bool read_vbus_adc(uint adc_chan)
{
    uint32_t acc = 0;
    adc_select_input(adc_chan);
    for (int i = 0; i < 8; i++)
    {
        acc += adc_read();
    }
    uint16_t code = acc / 8;
    uint16_t mv = adc_mv(code);

    // Print
    print_fmt("ADC%d: code=%d, VBUS=%dmV\n", adc_chan, code, mv);
    return mv >= 3800;
}

void scan_present_ports()
{
    g_port_map = 0;
    g_port_count = 0;
    for (uint8_t port = 0; port < MAX_PORTS; port++)
    {
        if (gpio_get(port == 0 ? PORT_0_VBUS_SENSE_PIN : PORT_1_VBUS_SENSE_PIN))
        {
            print_fmt("Detected VBUS on port %d\n", port);
            g_port_map |= (1 << port);
            g_port_count++;
        }
    }

    print_fmt("Detected %d port(s)\n", g_port_count);
}

bool probe_port(uint8_t port)
{
    usb_switch_to_port(port);
    uint32_t t0 = board_millis();
    while (board_millis() - t0 < 1000)
    {
        tud_task();
        if (g_mounted)
        {
            return true;
        }
    }

    return false;
}

void tud_mount_cb(void)
{
    g_mounted = true;

    print_fmt("USB mounted port %d\n", g_active_port);
}

void tud_umount_cb(void)
{
    g_mounted = false;
    print_fmt("USB unmounted port %d\n", g_active_port);
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

enum
{
    REQ_GET_PORT = 0x01,
    REQ_SET_PORT = 0x02,
    REQ_GET_POWER = 0x03,
    REQ_GET_PORTMAP = 0x10,
};

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP)
    {
        if (stage == CONTROL_STAGE_ACK && g_pending_switch)
        {
            g_switch_request_finished = true;
        }
        return true;
    }

    print_fmt("Vendor control request: 0x%02x", request->bRequest);
    print_fmt("  wValue: 0x%04x\n", request->wValue);

    switch (request->bRequest)
    {
    case REQ_GET_PORT:
    {
        // GET_PORT
        print_fmt("Reporting current port: %d\n", g_active_port);

        bool result = tud_control_xfer(rhport, request, &g_active_port, 1);
        return result;
    }
    case REQ_SET_PORT:
    {
        // SET_PORT
        g_pending_port = (uint8_t)request->wValue;
        g_pending_switch = true;
        g_switch_request_finished = false;
        return tud_control_status(rhport, request); // send status response
    }
    case REQ_GET_POWER:
    {
        // GET_POWER
        uint8_t power = 200; // mA
        return tud_control_xfer(rhport, request, &power, 1);
    }
    case REQ_GET_PORTMAP:
    {
        // GET_PORTMAP
        scan_present_ports();
        return tud_control_xfer(rhport, request, &g_port_map, 1);
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
        uint32_t count = tud_vendor_read(g_vnd_buffer + g_vnd_buflen, sizeof(g_vnd_buffer) - g_vnd_buflen);
        g_vnd_buflen += count;

        // If we have not parsed a header yet and have enough bits in the buffer
        if (g_vnd_expected_packet_size == 0 && g_vnd_buflen >= 6)
        {
            // printf("Parsing header: ");
            // Parse little endian
            uint32_t seq = (uint32_t)g_vnd_buffer[0] | ((uint32_t)g_vnd_buffer[1] << 8) |
                           ((uint32_t)g_vnd_buffer[2] << 16) | ((uint32_t)g_vnd_buffer[3] << 24);
            uint16_t len = (uint16_t)g_vnd_buffer[4] | ((uint16_t)g_vnd_buffer[5] << 8);

            // printf("seq: %d, size: %d\n", seq, len);

            g_vnd_expected_packet_size = (uint32_t)len;
        }

        if (g_vnd_expected_packet_size && g_vnd_buflen >= g_vnd_expected_packet_size)
        {
            // printf("Received enough bytes, sending packet\n");
            tud_vendor_write(g_vnd_buffer, g_vnd_expected_packet_size);
            tud_vendor_flush();

            uint32_t remain = g_vnd_buflen - g_vnd_expected_packet_size;
            memmove(g_vnd_buffer, g_vnd_buffer + g_vnd_buflen, remain);
            g_vnd_buflen = remain;
            g_vnd_expected_packet_size = 0;
        }
    }
}

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
        print_fmt("Current USB MUX port: %d\n", g_active_port);
    }
    else if (strcmp(cmd, "rst") == 0)
    {
        print_fmt("Restarting...\n");
        sleep_ms(100); // Give time for message to be sent
        // Restart without entering bootloader
        watchdog_reboot(0, 0, 0);
    }
    else if (strcmp(cmd, "prg") == 0)
    {
        print_fmt("Rebooting into bootloader...\n");
        sleep_ms(100); // Give time for message to be sent
        reset_usb_boot(0, 0);
    }
    else
    {
        print_fmt("Unknown command\n");
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
            if (g_cmd_buflen == 0)
                return; // Ignore empty commands
            // End of command
            g_cmd_buffer[g_cmd_buflen] = 0; // Null-terminate the string
            g_cmd_buflen = 0;               // Reset buffer length for next command
            uart_puts(UART_ID, "\n");       // New line after command

            // Process command
            process_command(g_cmd_buffer);
        }
        else if (ch == 8 || ch == 127)
        {
            if (g_cmd_buflen > 0)
            {
                // Handle backspace/delete: remove last char and erase it on the terminal
                g_cmd_buflen--;
                // uart_putc(UART_ID, '\b');
                // uart_putc(UART_ID, ' ');
                // uart_putc(UART_ID, '\b');
            }
        }
        else if (g_cmd_buflen < CMD_BUFFER_SIZE - 1)
        {
            // Store character in command buffer
            g_cmd_buffer[g_cmd_buflen++] = ch;
        }
    }
}
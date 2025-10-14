#include <stdlib.h>
#include <math.h>
#include <bsp/board_api.h>
#include <tusb.h>

#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>

#include <hardware/uart.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/watchdog.h>
#include <hardware/pwm.h>

#include "include.h"

static uint8_t g_vnd_buffer[2048];
static uint16_t g_vnd_buflen = 0;
static uint16_t g_vnd_expected_packet_size = 0;

static uint8_t g_active_port = 0;
static uint8_t g_port_map = 0;   // Bitmap of which ports have been connected to the host (VBUS present)
static uint8_t g_port_count = 0; // Number of enumerated ports
static bool g_mounted = false;

static uint8_t g_pending_port = 0;

static char g_cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t g_cmd_buflen = 0;

static uint16_t g_current_load = 0;

static int g_dma_chan = -1;
static uint16_t *g_adc_sample_buf = NULL;
static uint32_t g_adc_samples_per_ms = 0;
static uint32_t g_adc_samples_per_window = 0;
static uint32_t g_adc_samples_transient = 0;

static power_report_t g_power_report;

void service_vendor();
void on_uart_rx();
void read_present_ports();
void pwm_set_duty_frac(uint8_t pin, float frac);

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
        uart_puts(UART_ID, "\r\n");
    }
}

void init_pwm()
{
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);

    pwm_set_wrap(slice, PWM_WRAP);
    // pwm_set_clkdiv(slice, 1.0f);
    pwm_set_enabled(slice, true);

    pwm_set_duty_frac(PWM_PIN, 0);
}

void init_gpio()
{
    // GPIO pin for USB mux selection
    gpio_init(USB_MUX_SEL0_PIN);
    gpio_set_dir(USB_MUX_SEL0_PIN, GPIO_OUT);
    gpio_put(USB_MUX_SEL0_PIN, 0);
    gpio_init(USB_MUX_SEL1_PIN);
    gpio_set_dir(USB_MUX_SEL1_PIN, GPIO_OUT);
    gpio_put(USB_MUX_SEL1_PIN, 0);

    // GPIO pins for switching VBUS
    for (uint8_t i = 0; i < MAX_PORTS; i++)
    {
        uint8_t pin = PORT_VBUS_SWITCH_PINS[i];
        if (pin != 0)
        {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            gpio_put(pin, 0); // Start with VBUS off
        }
    }
}

void init_uart()
{
    // Set up UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // Set up an interrupt on RX
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    print_fmt("\nUART initialized");
}

// This is for the VBUS sensing and Current sensing
void init_adc_dma()
{
    g_adc_samples_per_ms = ADC_SAMPLE_RATE_HZ / 1000u;
    g_adc_samples_per_window = g_adc_samples_per_ms * STEP_WINDOW_MS;
    g_adc_samples_transient = g_adc_samples_per_ms * TRANSIENT_MS;

    g_adc_sample_buf = (uint16_t *)calloc(g_adc_samples_per_window, sizeof(uint16_t));

    adc_init();
    adc_gpio_init(VBUS_ADC_PIN);
    adc_select_input(VBUS_ADC_CHAN);

    // FIFO: enable, DREQ on, 12-bit right aligned, no error bit
    adc_fifo_setup(true, true, 1, false, false);

    // Set ADC clock divider to get the required sample rate:
    // ADC clock 48 MHz / (1 + div) = SAMPLE_RATE_HZ
    // => div = (48e6 / SAMPLE_RATE_HZ) - 1
    float div = 48000000.0f / (float)ADC_SAMPLE_RATE_HZ - 1.0f;
    if (div < 0.0f)
        div = 0.0f;
    adc_set_clkdiv(div);

    // DMA channel
    g_dma_chan = dma_claim_unused_channel(true);
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
    init_pwm();

    init_adc_dma();

    watchdog_enable(4000, 1); // 2 second timeout
    // main run loop
    while (1)
    {
        // TinyUSB device task | must be called regularly
        tud_task();

        service_vendor();

        watchdog_update();
    }

    // indicate no error
    return 0;
}

void usb_switch_to_port(uint8_t port)
{
    if (port > MAX_PORTS)
        return; // Invalid port

    tud_disconnect();
    sleep_ms(3); // Wait for host to notice disconnection

    gpio_put(USB_MUX_SEL0_PIN, port & 1);
    gpio_put(USB_MUX_SEL1_PIN, port & 2);

    sleep_ms(5); // Wait for the mux to switch
    tud_connect();

    print_fmt("Switched USB MUX to %d%d port %d", port & 2, port & 1, port);
    g_active_port = port;
}

void pwm_set_duty_frac(uint8_t pin, float frac)
{
    if (frac < 0.0f)
        frac = 0.0f;
    if (frac > 1.0f)
        frac = 1.0f;

    uint slice = pwm_gpio_to_slice_num(pin);

    uint16_t level = (uint16_t)lroundf(frac * PWM_WRAP);

    pwm_set_gpio_level(pin, level);

    print_fmt("Set PWM duty cycle to %.1f", frac * 100);
}

void load_set_mA(uint16_t mA)
{
    // Because of lifted ground issues, 100% duty cycle is closer to 450mA than 500mA
    // therefore 500/450 = 1.11 scale factor
    // I needed a bit extra to compensate so 1.2 :)
    float duty = (mA / 500.0f) * 1.2f;

    if (duty > 1.0f)
        duty = 1.0f;

    pwm_set_duty_frac(PWM_PIN, duty);
    g_current_load = mA;
}

void load_ramp_to_mA(uint16_t target_mA, uint32_t ramp_us)
{
    const uint32_t steps = ramp_us / 1000u + 1u;
    int32_t step_delta = ((int32_t)target_mA - (int32_t)g_current_load) / (int32_t)steps;
    print_fmt("Target: %d, current:%d, steps:%d, delta:%d", target_mA, g_current_load, steps, step_delta);
    for (uint32_t i = 1; i <= steps; i++)
    {
        uint16_t mA = g_current_load + step_delta;
        load_set_mA(mA);
        sleep_us(1000);
    }
}

uint16_t adc_code_to_mV(uint16_t code)
{
    return (uint16_t)VREF_MV * code / ADC_MAX;
}

uint16_t convert_mv_res_divider(uint16_t mv, uint16_t R1, uint16_t R2)
{
    float divider_scale = (float)(R1 + R2) / (float)R2;

    return (uint16_t)(mv * divider_scale);
}

uint16_t adc_code_to_vbus_mV(uint16_t code)
{
    return convert_mv_res_divider(adc_code_to_mV(code), VBUS_DIV_R1, VBUS_DIV_R2);
}

uint16_t read_adc_mv(uint channel)
{
    adc_select_input(channel);
    uint32_t acc = 0;
    uint8_t samples = 32;
    for (int i = 0; i < samples; i++)
        acc += adc_read();
    uint16_t avg = (uint16_t)(acc / samples);

    uint16_t mv = adc_code_to_mV(avg);

    print_fmt("CHAN%d: %dmV", channel, mv);

    return mv;
}

uint16_t read_vbus_mv()
{
    uint16_t mv = convert_mv_res_divider(read_adc_mv(VBUS_ADC_CHAN), VBUS_DIV_R1, VBUS_DIV_R2);

    // Print
    print_fmt("VBUS: %dmV", mv);
    return mv;
}

uint16_t read_current_mA()
{
    uint16_t mv = read_adc_mv(CURRENT_MEAS_ADC_CHAN);

    uint16_t mA = mv / (0.51f * 13.2f);

    print_fmt("CURRENT: %dmA", mA);
    return mA;
}

void vbus_turn_off()
{
    for (int port = 0; port < MAX_PORTS; port++)
    {
        if (PORT_VBUS_SWITCH_PINS[port] != 0)
        {
            gpio_put(PORT_VBUS_SWITCH_PINS[port], 0);
        }
    }
}

void vbus_set_activated(uint8_t port)
{
    vbus_turn_off();

    if (PORT_VBUS_SWITCH_PINS[port] == 0)
    {
        print_fmt("can't set port %d as activated VBUS", port);
        return;
    }

    sleep_us(100);

    gpio_put(PORT_VBUS_SWITCH_PINS[port], 1);

    print_fmt("VBUS %d ON", port);

    sleep_us(100);
}

void read_present_ports()
{
    // Port 0 is always connected
    g_port_map = 1;
    g_port_count = 1;
    load_set_mA(0); // Make sure there is no load

    for (uint8_t port = 1; port < MAX_PORTS; port++)
    {
        if (PORT_VBUS_SWITCH_PINS[port] == 0)
        {
            continue;
        }

        vbus_set_activated(port);

        sleep_ms(50);

        if (read_vbus_mv() >= UNDERVOLT_LIMIT_MV)
        {
            print_fmt("Detected VBUS on port %d", port);
            g_port_map |= (1 << port);
            g_port_count++;
        }
    }

    vbus_turn_off();

    print_fmt("Detected %d port(s)", g_port_count);
}

bool usb_probe_port(uint8_t port)
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

int64_t usb_do_port_switch(alarm_id_t id, void *user_data)
{
    (void)id;
    (void)user_data;

    read_present_ports();

    if (g_pending_port != g_active_port)
    {
        if ((g_port_map & (1 << g_pending_port)) == 0)
        {
            print_fmt("Port %d not available, ignoring switch request", g_pending_port);
        }
        else
        {
            print_fmt("Switching to port %d as requested by host", g_pending_port);
            usb_switch_to_port(g_pending_port);
        }
    }
    else
    {
        print_fmt("Already on port %d, ignoring switch request", g_pending_port);
    }

    return 0;
}

// ------------------ POWER TESTING -----------------------

static inline uint16_t clamp_u16(int v)
{
    if (v < 0)
        return 0;
    if (v > 0xFFFF)
        return 0xFFFF;
    return (uint16_t)v;
}

void compute_stats_from_run(adc_capture_stats_t *s_out)
{
    // Convert on the fly; do two passes:
    // Pass 1: transient (0..g_adc_samples_transient-1) for v_min & recovery
    // Pass 2: steady (g_adc_samples_transient..n-1) for mean and ripple
    uint32_t steady_vmin = 0xFFFFFFFFu;
    uint32_t steady_vmax = 0;

    // Steady window
    uint32_t steady_sample_count = g_adc_samples_per_window - g_adc_samples_transient;
    uint64_t sum = 0;

    for (uint32_t i = g_adc_samples_transient; i < g_adc_samples_per_window; ++i)
    {
        uint32_t mv = adc_code_to_vbus_mV(g_adc_sample_buf[i]);
        sum += mv;
        if (mv < steady_vmin)
            steady_vmin = mv;
        if (mv > steady_vmax)
            steady_vmax = mv;
    }
    uint32_t total_vmin = steady_vmin;
    uint32_t total_vmax = steady_vmax;

    uint32_t mean = (uint32_t)(sum / (uint64_t)steady_sample_count);
    uint32_t ripple = steady_vmax - steady_vmin;

    // Recovery time: first index in transient where VBUS >= (mean - VRECOV_THRESH_MV)
    uint32_t thresh = mean - VRECOV_THRESH_MV;
    uint32_t rec_idx = g_adc_samples_transient; // default: within steady window
    for (uint32_t i = 0; i < g_adc_samples_transient && i < g_adc_samples_per_window; ++i)
    {
        uint32_t mv = adc_code_to_vbus_mV(g_adc_sample_buf[i]);
        if (mv < total_vmin)
            total_vmin = mv;
        if (mv > total_vmax)
            total_vmax = mv;

        if (mv >= thresh)
        {
            rec_idx = i;
            break;
        }
    }
    uint16_t rec_us = (rec_idx * 1000000u) / ADC_SAMPLE_RATE_HZ;

    s_out->v_min_mV = clamp_u16((int)total_vmin);
    s_out->v_max_mV = clamp_u16((int)total_vmax);
    s_out->v_mean_mV = clamp_u16((int)mean);
    s_out->ripple_mVpp = clamp_u16((int)ripple);
    s_out->recovery_us = clamp_u16((int)rec_us);
}

void adc_dma_capture_window_blocking()
{
    dma_channel_config c = dma_channel_get_default_config(g_dma_chan);

    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

    dma_channel_configure(g_dma_chan, &c,
                          g_adc_sample_buf,         // write addr
                          &adc_hw->fifo,            // read addr
                          g_adc_samples_per_window, // number of samples
                          false);
    adc_select_input(VBUS_ADC_CHAN);
    adc_fifo_drain();
    adc_run(false);
    adc_fifo_setup(true, true, 1, false, false);
    adc_run(true);
    dma_start_channel_mask(1u << g_dma_chan);

    // Wait until done
    dma_channel_wait_for_finish_blocking(g_dma_chan);
    adc_run(false);
}

void run_power_test(power_report_t *out_report)
{

    uint8_t steps_count = 5;
    uint16_t steps_mA[5] = {100, 200, 300, 400, 500};

    load_set_mA(0);

    memset(out_report, 0, sizeof(*out_report));

    out_report->port = g_active_port;
    out_report->n_steps = steps_count;
    out_report->maxpower_mA = 500;

    if (PORT_VBUS_SWITCH_PINS[g_active_port] == 0)
    {
        print_fmt("Can't run power test on port %d", g_active_port);
        out_report->flags |= (1u << 2); // cant_load
        return;
    }

    vbus_set_activated(g_active_port);

    adc_dma_capture_window_blocking();

    adc_capture_stats_t s0;
    compute_stats_from_run(&s0);
    out_report->v_idle_mV = s0.v_mean_mV;

    print_fmt("IDLE: %dmV", s0.v_mean_mV);

    // If VBUS missing, flag & bail with partial report
    if (out_report->v_idle_mV < UNDERVOLT_LIMIT_MV)
    {
        print_fmt("VBUS too low, exiting");
        out_report->flags |= (1u << 0); // vbus_missing
        return;
    }

    uint16_t max_current_ok = 0;
    bool ocp = false;
    uint16_t ocp_at = 0;

    for (uint8_t i = 0; i < steps_count; i++)
    {
        uint16_t req_mA = steps_mA[i];
        out_report->loads_mA[i] = req_mA;

        load_ramp_to_mA(req_mA, 2000);

        adc_dma_capture_window_blocking();

        adc_capture_stats_t s;
        compute_stats_from_run(&s);
        uint16_t current = read_current_mA();

        out_report->v_min_mV[i] = s.v_min_mV;
        out_report->v_max_mV[i] = s.v_max_mV;
        out_report->v_mean_mV[i] = s.v_mean_mV;
        out_report->ripple_mVpp[i] = s.ripple_mVpp;
        out_report->droop_mV[i] = (out_report->v_idle_mV > s.v_min_mV) ? (out_report->v_idle_mV - s.v_min_mV) : 0;
        out_report->recovery_us[i] = s.recovery_us;
        out_report->current_mA[i] = current;

        print_fmt("LOAD: %dmA, V: %dmV", current, s.v_mean_mV);

        if (s.v_mean_mV < UNDERVOLT_LIMIT_MV)
        {
            ocp = true;
            ocp_at = req_mA;
            break;
        }

        max_current_ok = current;
    }

    load_set_mA(0);
    vbus_turn_off();

    out_report->max_current_mA = max_current_ok;
    if (ocp)
    {
        out_report->flags |= (1u << 1); // ocp
        out_report->ocp_at_mA = ocp_at;
    }

    print_fmt("Finished");
}

// ------------------ USB CALLBACKS -----------------------

void tud_mount_cb(void)
{
    g_mounted = true;

    print_fmt("USB mounted port %d", g_active_port);
}

void tud_umount_cb(void)
{
    g_mounted = false;
    print_fmt("USB unmounted port %d", g_active_port);
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
        if (stage == CONTROL_STAGE_ACK && request->bRequest == REQ_SET_PORT)
        {
            add_alarm_in_ms(5, usb_do_port_switch, NULL, false);
        }
        return true;
    }

    print_fmt("Vendor control request: 0x%02x   wValue: 0x%04x", request->bRequest, request->wValue);

    switch (request->bRequest)
    {
    case REQ_GET_PORT:
    {
        // GET_PORT
        print_fmt("Reporting current port: %d", g_active_port);

        bool result = tud_control_xfer(rhport, request, &g_active_port, 1);
        return result;
    }
    case REQ_SET_PORT:
    {
        // SET_PORT
        g_pending_port = (uint8_t)request->wValue;
        return tud_control_status(rhport, request); // send status response
    }
    case REQ_GET_POWER:
    {
        // GET_POWER
        memset(&g_power_report, 0, sizeof(g_power_report));
        run_power_test(&g_power_report);
        _Static_assert(sizeof(power_report_t) == 93, "power_report_t size changed!"); // This is because the size is hardcoded in python
        return tud_control_xfer(rhport, request, &g_power_report, sizeof(g_power_report));
    }
    case REQ_GET_PORTMAP:
    {
        // GET_PORTMAP
        read_present_ports();
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
            // Parse little endian
            uint32_t seq = (uint32_t)g_vnd_buffer[0] | ((uint32_t)g_vnd_buffer[1] << 8) |
                           ((uint32_t)g_vnd_buffer[2] << 16) | ((uint32_t)g_vnd_buffer[3] << 24);
            uint16_t len = (uint16_t)g_vnd_buffer[4] | ((uint16_t)g_vnd_buffer[5] << 8);

            // print_fmt("Parsing header: seq: %d, size: %d", seq, len);

            g_vnd_expected_packet_size = (uint32_t)len;
        }

        if (g_vnd_expected_packet_size && g_vnd_buflen >= g_vnd_expected_packet_size)
        {
            // print_fmt("Received enough bytes, sending packet");
            tud_vendor_write(g_vnd_buffer, g_vnd_expected_packet_size);
            tud_vendor_flush();

            uint32_t remain = g_vnd_buflen - g_vnd_expected_packet_size;
            memmove(g_vnd_buffer, g_vnd_buffer + g_vnd_buflen, remain);
            g_vnd_buflen = remain;
            g_vnd_expected_packet_size = 0;
        }
    }
}

// ------------------- UART -------------------------

void process_command(const char *cmd)
{
    if (strcmp(cmd, "set 1") == 0)
    {
        usb_switch_to_port(1);
    }
    else if (strcmp(cmd, "set 2") == 0)
    {
        usb_switch_to_port(2);
    }
    else if (strcmp(cmd, "set 0") == 0)
    {
        usb_switch_to_port(0);
    }
    else if (strcmp(cmd, "get") == 0)
    {
        print_fmt("Current USB MUX port: %d", g_active_port);
    }
    else if (strcmp(cmd, "vbus 1") == 0)
    {
        vbus_set_activated(1);
    }
    else if (strcmp(cmd, "vbus off") == 0)
    {
        vbus_turn_off();
    }
    else if (strcmp(cmd, "pwr") == 0)
    {
        memset(&g_power_report, 0, sizeof(g_power_report));
        run_power_test(&g_power_report);
    }
    else if (strcmp(cmd, "sense") == 0)
    {
        read_present_ports();
    }
    else if (strcmp(cmd, "volt") == 0)
    {
        adc_select_input(VBUS_ADC_CHAN);
        print_fmt("%dmV", adc_code_to_vbus_mV(adc_read()));
    }
    else if (strcmp(cmd, "current") == 0)
    {
        read_current_mA();
    }
    else if (strcmp(cmd, "load 0") == 0)
    {
        load_set_mA(0);
    }
    else if (strcmp(cmd, "load 100") == 0)
    {
        load_set_mA(100);
    }
    else if (strcmp(cmd, "load 200") == 0)
    {
        load_set_mA(200);
    }
    else if (strcmp(cmd, "load 300") == 0)
    {
        load_set_mA(300);
    }
    else if (strcmp(cmd, "load 400") == 0)
    {
        load_set_mA(400);
    }
    else if (strcmp(cmd, "load 500") == 0)
    {
        load_set_mA(500);
    }
    else if (strcmp(cmd, "rst") == 0)
    {
        print_fmt("Restarting...");
        sleep_ms(100); // Give time for message to be sent
        // Restart without entering bootloader
        watchdog_reboot(0, 0, 0);
    }
    else if (strcmp(cmd, "prg") == 0)
    {
        print_fmt("Rebooting into bootloader...");
        sleep_ms(100); // Give time for message to be sent
        reset_usb_boot(0, 0);
    }
    else
    {
        print_fmt("Unknown command");
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
            uart_puts(UART_ID, "\r\n");     // New line after command

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
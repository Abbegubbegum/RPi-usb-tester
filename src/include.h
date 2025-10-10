#pragma once
// ---------- USB STUFF ----------
// set some example Vendor and Product ID
// the board will use to identify at the host
#define DEVICE_VID 0x1209 // Generic
#define DEVICE_PID 0x4004
// set USB 2.0
#define DEVICE_BCD 0x0200

#define ENDPOINT_BULK_SIZE 64

enum
{
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
    ITF_NUM_VENDOR,
    ITF_NUM_TOTAL
};

// define endpoint numbers
#define EPNUM_CDC_0_NOTIF 0x81 // notification endpoint for CDC 0
#define EPNUM_CDC_0_OUT 0x01   // out endpoint for CDC 0
#define EPNUM_CDC_0_IN 0x82    // in endpoint for CDC 0

#define EPNUM_VENDOR_OUT 0x05
#define EPNUM_VENDOR_IN 0x85

// ------------------------

#define USB_MUX_SEL_PIN 20

#define UART_ID uart0
#define BAUD_RATE 115200u

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define MAX_PORTS 2

static const uint8_t PORT_SENSE_PINS[] = {
    14,
    15};

static const uint8_t PORT_VBUS_SWITCH_PINS[] = {
    0, 13};

#define VBUS_ADC_PIN 26
#define VBUS_ADC_CHAN 0
#define VBUS_DIV_R1 675u
#define VBUS_DIV_R2 990u

#define CMD_BUFFER_SIZE 32

#define VREF_MV 3300u
#define ADC_MAX 4095u

#define PWM_WRAP 499u
#define PWM_PIN 4

#define CURRENT_MEAS_PIN 27
#define CURRENT_MEAS_ADC_CHAN 1

// Power testing
#define ADC_SAMPLE_RATE_HZ 80000u // 80 kS/s
#define STEP_WINDOW_MS 120u       // per load step
#define TRANSIENT_MS 20u          // first 20 ms used for droop/recovery
#define VRECOV_THRESH_MV 10       // recovery threshold from mean (10 mV)
#define UNDERVOLT_LIMIT_MV 4350   // “USB unhealthy” floor

//  ========= TYPES =============
typedef struct __attribute__((packed))
{
    uint8_t port;
    uint8_t n_steps;
    uint8_t flags;
    uint16_t maxpower_mA;
    uint16_t v_idle_mV;
    uint16_t loads_mA[4];
    uint16_t v_mean_mV[4];
    uint16_t v_min_mV[4];
    uint16_t droop_mV[4];
    uint16_t ripple_mVpp[4];
    uint32_t recovery_us[4];
    uint16_t max_current_mA;
    uint16_t ocp_at_mA;
    uint16_t errors;
} power_report_t;

typedef struct
{
    uint16_t v_min_mV;
    uint16_t v_max_mV;
    uint16_t v_mean_mV;
    uint16_t ripple_mVpp;
    uint32_t recovery_us;
} adc_capture_stats_t;
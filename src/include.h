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

#define USB_MUX_SEL0_PIN 13
#define USB_MUX_SEL1_PIN 14

#define UART_ID uart0
#define BAUD_RATE 115200u

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define MAX_PORTS 4

static const uint8_t PORT_VBUS_SWITCH_PINS[] = {
    0, 11, 10, 9};

static const uint8_t PORT_PASSED_LED_PINS[] = {
    18, 19, 20, 21};

#define BUZZER_EN_PIN 5

#define VBUS_ADC_PIN 26
#define VBUS_ADC_CHAN 0
#define VBUS_DIV_R1 68u
#define VBUS_DIV_R2 100u

#define CMD_BUFFER_SIZE 32

#define VREF_MV 3300u
#define ADC_MAX 4095u

#define PWM_WRAP 499u
#define PWM_PIN 3

#define CURRENT_MEAS_PIN 27
#define CURRENT_MEAS_ADC_CHAN 1

// Power testing measurment
#define STEP_WINDOW_MS 120u       // Measurment time per load step
#define TRANSIENT_MS 20u          // first 20 ms used for droop/recovery
#define ADC_SAMPLE_RATE_HZ 80000u // 80 kS/s
#define ADC_SAMPLE_PER_MS (ADC_SAMPLE_RATE_HZ / 1000u)
#define ADC_SAMPLES_PER_WINDOW (ADC_SAMPLE_PER_MS * STEP_WINDOW_MS)
#define ADC_TRANSIENT_SAMPLE_COUNT (ADC_SAMPLE_PER_MS * TRANSIENT_MS)

#define UNDERVOLT_LIMIT_IDLE_MV 4800 // Minimum VBUS to consider port functional
#define UNDERVOLT_LIMIT_LOAD_MV 4000 // Limit for the drop during load, more lenient because of resistance in load path

#define VBUS_CHECK_INTERVAL_MS 1000 // Check for new ports every 1 second

//  ========= TYPES =============
typedef struct __attribute__((packed))
{
    uint8_t port;
    uint8_t n_steps;
    uint8_t flags; // bit 0: vbus_missing, bit 1: undervolt
    uint16_t v_idle_mV;
    uint16_t load_pct[5]; // Load percentage (20%, 40%, 60%, 80%, 100% where 100% = 500mA)
    uint16_t v_mean_mV[5];
    uint16_t v_min_mV[5];        // Minimum voltage during load (includes transient)
    uint16_t droop_mV[5];        // v_idle - v_min (total voltage drop)
    uint16_t ripple_mVpp[5];     // Voltage noise during steady-state
    uint16_t current_mA[5];      // Measured current draw
    uint16_t resistance_mOhm[5]; // Calculated resistance: droop_mV / current_mA (in milliohms)
    uint16_t max_current_mA;     // Highest current successfully delivered
    uint16_t undervolt_at_pct;   // Load where undervolt occurred
    uint16_t errors;
} power_report_t;

typedef struct
{
    uint16_t v_min_mV;
    uint16_t v_mean_mV;
    uint16_t ripple_mVpp;
} adc_capture_stats_t;
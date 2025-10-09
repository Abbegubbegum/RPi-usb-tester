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
#define BAUD_RATE 115200

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define MAX_PORTS 2

#define PORT_0_VBUS_SENSE_PIN 14

#define PORT_1_VBUS_SENSE_PIN 15
#define PORT_1_VBUS_SWITCH_PIN 13

#define VBUS_ADC_CHAN 0
#define VBUS_DIV_R1 675
#define VBUS_DIV_R2 990

#define CMD_BUFFER_SIZE 32

#define VREF_MV 3300
#define ADC_MAX 4095
// DIV R1 = 180k, R2 = 68k
#define DIV_RATIO_X100 ((180 + 68) / 68) * 100
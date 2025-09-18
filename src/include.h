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
// set some example Vendor and Product ID
// the board will use to identify at the host
#define DEVICE_VID 0xCafe
// use _PID_MAP to generate unique PID for each interface
#define DEVICE_PID (0x4004)
// set USB 2.0
#define DEVICE_BCD 0x0200

#define MAX_ENDPOINT0_SIZE 64
#define ENDPOINT_BULK_SIZE 64

enum
{
    STRID_LANGID = 0,   // 0: supported language ID
    STRID_MANUFACTURER, // 1: Manufacturer
    STRID_PRODUCT,      // 2: Product
    STRID_SERIAL,       // 3: Serials
    STRID_CDC_0,        // 4: CDC Interface 0
    STRID_CDC_1,        // 5: CDC Interface 1
    STRID_VENDOR,       // 6: Vendor Interface
};

enum
{
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
    ITF_NUM_CDC_1,
    ITF_NUM_CDC_1_DATA,
    ITF_NUM_VENDOR,
    ITF_NUM_TOTAL
};

// define endpoint numbers
#define EPNUM_CDC_0_NOTIF 0x81 // notification endpoint for CDC 0
#define EPNUM_CDC_0_OUT 0x01   // out endpoint for CDC 0
#define EPNUM_CDC_0_IN 0x82    // in endpoint for CDC 0

#define EPNUM_CDC_1_NOTIF 0x83 // notification endpoint for CDC 1
#define EPNUM_CDC_1_OUT 0x03   // out endpoint for CDC 1
#define EPNUM_CDC_1_IN 0x84    // in endpoint for CDC 1

#define EPNUM_VENDOR_OUT 0x05
#define EPNUM_VENDOR_IN 0x85
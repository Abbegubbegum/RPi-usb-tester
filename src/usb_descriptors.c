/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <tusb.h>
#include <bsp/board_api.h>

#include "include.h"

// defines a descriptor that will be communicated to the host
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = DEVICE_BCD,

    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = DEVICE_VID,
    .idProduct = DEVICE_PID,
    .bcdDevice = 0x0100, // Device release number

    .iManufacturer = 0x01, // Index of manufacturer string
    .iProduct = 0x02,      // Index of product string
    .iSerialNumber = 0x03, // Index of serial number string

    .bNumConfigurations = 0x01 // 1 configuration
};

// called when host requests to get device descriptor
uint8_t const *tud_descriptor_device_cb(void);

// total length of configuration descriptor
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN)

// configure descriptor (for 2 CDC interfaces)
uint8_t const desc_configuration[] = {
    // config descriptor | how much power in mA, count of interfaces, ...
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

    // CDC 0: Communication Interface - TODO: get 64 from tusb_config.h
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),
    // CDC 0: Data Interface
    // TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0_DATA, 4, 0x01, 0x02),

    // Vendor (class 0xFF)
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 5, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, ENDPOINT_BULK_SIZE),
};

// called when host requests to get configuration descriptor
uint8_t const *tud_descriptor_configuration_cb(uint8_t index);

// more device descriptor this time the qualifier
tusb_desc_device_qualifier_t const desc_device_qualifier = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = DEVICE_BCD,

    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0x00};

// called when host requests to get device qualifier descriptor
uint8_t const *tud_descriptor_device_qualifier_cb(void);

// String descriptors referenced with .i... in the descriptor tables
enum
{
    STRID_LANGID = 0,   // 0: supported language ID
    STRID_MANUFACTURER, // 1: Manufacturer
    STRID_PRODUCT,      // 2: Product
    STRID_SERIAL,       // 3: Serials
    STRID_CDC_0,        // 4: CDC Interface 0
    STRID_VENDOR,       // 5: Vendor Interface
};

// array of pointer to string descriptors
char const *string_desc_arr[] = {
    // switched because board is little endian
    (const char[]){0x09, 0x04}, // 0: supported language is English (0x0409)
    "FMTIS GA BODEN",           // 1: Manufacturer
    "RPico2 USB Tester",        // 2: Product
    NULL,                       // 3: Serials (null so it uses unique ID if available)
    "CDC0"                      // 4: CDC Interface 0
    "Test interface"            // 5: Vendor Interface
};

// called when host request to get string descriptor
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid);

// --------------------------------------------------------------------+
// IMPLEMENTATION
// --------------------------------------------------------------------+

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return (uint8_t const *)&desc_device_qualifier;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    // avoid unused parameter warning and keep function signature consistent
    (void)index;

    return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    // TODO: check lang id
    (void)langid;
    size_t char_count;
    static uint16_t _desc_str[32 + 1];
    const char *str;

    // Determine which string descriptor to return
    switch (index)
    {
    case STRID_LANGID:
        memcpy(&_desc_str[1], string_desc_arr[STRID_LANGID], 2);
        char_count = 1;
        break;

    case STRID_SERIAL:
        // try to read the serial from the board
        char_count = board_usb_get_serial(_desc_str + 1, 32);
        break;

    default:
        // COPYRIGHT NOTE: Based on TinyUSB example
        // Windows wants utf16le

        // Determine which string descriptor to return
        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
        {
            return NULL;
        }

        str = string_desc_arr[index];
        char_count = strlen(str);

        // Ensure we don't overwrite the buffer
        size_t const max_count = (sizeof(_desc_str) / sizeof(_desc_str[0])) - 1;
        if (char_count > max_count)
        {
            char_count = max_count;
        }

        // Convert to UTF-16
        for (size_t i = 0; i < char_count; i++)
        {
            _desc_str[1 + i] = str[i];
        }
        break;
    }

    // First byte is the length (including header), second byte is string type
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (char_count * 2 + 2));

    return _desc_str;
}

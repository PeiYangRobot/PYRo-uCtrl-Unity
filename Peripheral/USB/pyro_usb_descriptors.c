#include "tusb.h"
#include "pyro_usb_desc_config.h"
#include <string.h>

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
// 说明：VID/PID/BCD 与厂商/产品名均为项目级编译期常量（见 pyro_usb_desc_config.h）；
//       唯一可变项是序列号 —— 由应用层在启动 USB 栈之前注入
//       （见文件末尾 pyro_usb_desc_set_serial() 与 usb_cdc_drv_t::start(serial)）。
#define USB_VID   PYRO_USB_VID
#define USB_PID   PYRO_USB_PID
#define USB_BCD   PYRO_USB_BCD

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = USB_BCD,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

// Invoked when received GET DEVICE DESCRIPTOR
uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor（单 CDC；FS-only，无需 HS / OTHER_SPEED）
//--------------------------------------------------------------------+
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power(mA)
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    // Interface number, string index, EP notif addr & size, EP out, EP in, EP size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_fs_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC_ITF
};

/* 注意：数组本身可写 —— 序列号（index 3）由应用层经 pyro_usb_desc_set_serial() 注入；
 *       其余各项均为编译期常量，且厂商/产品名**不得含机型字样**。 */
static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },   // 0: English (0x0409)
    PYRO_USB_MANUFACTURER_STR,       // 1: Manufacturer
    PYRO_USB_PRODUCT_STR,            // 2: Product
    "PYRO",                          // 3: Serial（占位值；实际值由应用层经 set_serial 注入）
    PYRO_USB_CDC_ITF_STR,            // 4: CDC interface
};

/* 注入序列号（时序要求见 pyro_usb_desc_config.h；此处只做最小防御） */
void pyro_usb_desc_set_serial(const char *s) {
    if (s != NULL && s[0] != '\0') {
        string_desc_arr[STRID_SERIAL] = s;
    }
}

// 设备描述符字符串
static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t) strlen(str);
        if (chr_count > PYRO_USB_SERIAL_MAX_LEN) {
            chr_count = (uint8_t) PYRO_USB_SERIAL_MAX_LEN;
        }
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];   // ASCII -> UTF-16LE
        }
    }

    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
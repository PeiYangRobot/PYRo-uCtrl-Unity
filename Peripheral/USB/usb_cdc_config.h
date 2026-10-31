#ifndef __USB_CDC_CONFIG_H__
#define __USB_CDC_CONFIG_H__

namespace usb_cdc {
    constexpr uint16_t MAX_DATA_LENGTH = 256;
    constexpr uint8_t SOF = 0xA5;
    constexpr uint8_t HEADER_SIZE = 5;

    constexpr uint16_t RX_BUF_CAPACITY = 640, TX_BUF_CAPACITY = 1024;
}

#endif // __USB_CDC_CONFIG_H__

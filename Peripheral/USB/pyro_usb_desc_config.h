#ifndef __PYRO_USB_DESC_CONFIG_H__
#define __PYRO_USB_DESC_CONFIG_H__

/**
 * @file pyro_usb_desc_config.h
 * @brief USB 设备身份（描述符）配置：厂商/产品为项目级常量，序列号由应用层直接给出。
 *
 * 长度约束：TinyUSB 本项目的字符串缓冲为 _desc_str[32+1] → 最多 31 个字符；
 *          超长会在 start() 中被拒绝（PYRO_PARAM_ERROR），不做静默截断。
 */

/* 厂商 / 产品 / CDC 接口字符串（项目级常量，不含机型） */
#define PYRO_USB_MANUFACTURER_STR "PYRo"
#define PYRO_USB_PRODUCT_STR      "PYRo Robot Virtual COM"
#define PYRO_USB_CDC_ITF_STR      "PYRo CDC"

/* USB 字符串描述符上限：本项目 _desc_str[32+1] → 31 个字符 */
#define PYRO_USB_SERIAL_MAX_LEN 31

/* VID / PID / bcdDevice（编译期常量；正式值与 .inf 需求见 plan.md §6） */
#ifndef PYRO_USB_VID
#define PYRO_USB_VID 0xCAFE
#endif
#ifndef PYRO_USB_PID
#define PYRO_USB_PID 0x4010
#endif
#ifndef PYRO_USB_BCD
#define PYRO_USB_BCD 0x0100
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 注入设备序列号（字符串描述符 index 3）。
 * @param s 可打印 ASCII、非空、长度 ≤ PYRO_USB_SERIAL_MAX_LEN，且必须为静态存储
 *          （回调会被多次调用：重复 GET_DESCRIPTOR、重枚举）。
 * @note 由 usb_cdc_drv_t::start(serial) 在 tusb_init() 之前调用；
 *       传入 NULL/空串时忽略（保留占位值；正常流程不会发生：start() 已强制序列号非空）。
 */
void pyro_usb_desc_set_serial(const char *s);

#ifdef __cplusplus
}
#endif

#endif // __PYRO_USB_DESC_CONFIG_H__

#ifndef __PYRO_USB_CDC_TEST_H__
#define __PYRO_USB_CDC_TEST_H__

#include "pyro_core_def.h"
#include <cstdint>

/**
 * @file pyro_usb_cdc_test.h
 * @brief USB 虚拟串口（多 CDC 口）回环自测模块。
 *
 * 目的：在**没有上位机协议栈**的情况下，仅凭一个串口助手就能验证
 * 「帧路由 → 分发 → 回显」整条链路，以及多个 serial_id 之间互不串扰。
 *
 * ---------------------------------------------------------------------------
 * 用法
 * ---------------------------------------------------------------------------
 * 1) 在 pyro_init_thread.cpp 里 usb_cdc_drv_t::instance().init(...) 之后调用：
 *
 *      #if defined(USB_CDC_TEST)
 *          usb_cdc_test_enable();
 *      #endif
 *
 * 2) 编译时加 -DUSB_CDC_TEST=1（或 CMake 里 target_compile_definitions）。
 *
 * 3) 用串口助手按下面的帧格式发数据，模块会原样回显。
 *
 * ---------------------------------------------------------------------------
 * 占用的帧 ID（与业务帧 0x01/0x02 不冲突）
 * ---------------------------------------------------------------------------
 *   0x7B        identify：查询本端口身份，应答 12 B 的 identify_t
 *   0x7C~0x7F   echo    ：原样回显（frame_id 与载荷都保持不变）
 *
 * ---------------------------------------------------------------------------
 * 设计要点
 * ---------------------------------------------------------------------------
 * - **回显走原路**：从哪个 serial_id 进来，就从哪个 serial_id 出去。
 *   所以只要两个口各连一个串口助手，就能直接看出路由有没有错。
 * - **不注册重复 ID**：usb_cdc_drv_t::register_rx() 对重复 frame_id 返回
 *   PYRO_PARAM_ERROR 且不覆盖，所以 enable 失败时会把已注册的**全部回滚**。
 * - **handler 内不做阻塞**：回显 handler 只调 send()，不等待、不 sleep。
 *   send() 内部有 _tx_mutex（1 tick 超时），拿不到就返回 PYRO_BUSY 并丢帧，
 *   这是刻意的——handler 跑在 usb_task 上下文里，阻塞会拖垮整个 USB 栈。
 */

namespace pyro {
namespace usb_cdc_test {

/** 驱动里 serial_t 的个数，必须与 pyro_usb_cdc_drv.h 的 _serial[] 长度一致。 */
constexpr uint8_t SERIAL_COUNT = 2;

/** 查询端口身份。载荷内容被忽略，应答见 identify_t。 */
constexpr uint8_t FRAME_ID_IDENTIFY = 0x7B;

/** 回显帧 ID 区间起点，实际占用 [0x7C, 0x7C + FRAME_ID_ECHO_COUNT)。 */
constexpr uint8_t FRAME_ID_ECHO_BASE = 0x7C;

/** 回显帧 ID 个数。 */
constexpr uint8_t FRAME_ID_ECHO_COUNT = 4;

/** identify 应答的魔数，用于确认应答确实来自本模块。 */
constexpr uint8_t IDENTIFY_MAGIC = 0x5A;

/**
 * @brief identify 应答载荷（12 B，小端）。
 *
 * 上位机据此确认：COM 口 ↔ serial_id 的映射是否与预期一致。
 * 双串口最容易出错的地方就是两个口顺序反了，这个字段就是用来抓它的。
 */
struct identify_t {
    uint8_t serial_id;   ///< 本端口的 serial_id（0 或 1）
    uint8_t magic;       ///< 固定 IDENTIFY_MAGIC
    uint16_t reserved;   ///< 对齐填充，填 0
    uint32_t echo_count; ///< 本端口累计回显帧数（uint32，不会溢出）
    uint32_t tick_ms;    ///< xTaskGetTickCount() 换算的毫秒数
} __attribute__((packed));

static_assert(sizeof(identify_t) == 12, "identify_t 必须正好 12 B");

} // namespace usb_cdc_test

/**
 * @brief 在所有 serial_id 上注册回环 handler。
 *
 * 幂等：重复调用直接返回 PYRO_OK。
 *
 * @return PYRO_OK            全部注册成功
 * @return PYRO_PARAM_ERROR   帧 ID 已被业务占用（此时已回滚，不留半成品）
 * @return PYRO_NO_MEMORY     注册记录表溢出（不应发生）
 * @return 其他               驱动透传的错误码
 *
 * @note 必须在 usb_cdc_drv_t::init() **之后**调用——init() 内部会对每个
 *       serial_id 调 reset()，而 reset() 会清空 _rx_handlers。
 * @note 但**不需要**等 USB 挂载完成：handler 表在 mount/umount 时不会被清空，
 *       挂载前注册才是正确做法，否则上电瞬间的帧会丢。
 */
status_t usb_cdc_test_enable();

/**
 * @brief 注销所有回环 handler。
 *
 * @return PYRO_OK 成功（未启用时也返回 PYRO_OK）
 */
status_t usb_cdc_test_disable();

/**
 * @brief 查询当前是否已启用。
 */
bool usb_cdc_test_is_enabled();

/**
 * @brief 取某个 serial_id 的累计回显帧数（调试用）。
 */
uint32_t usb_cdc_test_echo_count(uint8_t serial_id);

} // namespace pyro

#endif // __PYRO_USB_CDC_TEST_H__

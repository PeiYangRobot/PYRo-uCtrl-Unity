#include "pyro_usb_cdc_test.h"

#include "pyro_usb_cdc_drv.h"

#include "FreeRTOS.h"
#include "task.h"

namespace pyro {
namespace {

using namespace usb_cdc_test;

/** 是否已启用。 */
bool g_enabled = false;

/** 每个 serial_id 的累计回显帧数。 */
uint32_t g_echo_count[SERIAL_COUNT] = {};

/** 已注册的 (serial_id, frame_id)，用于失败回滚。 */
struct reg_record_t {
    uint8_t serial_id;
    uint8_t frame_id;
};

constexpr uint8_t MAX_REGS = SERIAL_COUNT * (1 + FRAME_ID_ECHO_COUNT);
reg_record_t g_regs[MAX_REGS] = {};
uint8_t g_reg_count = 0;

/** 记录一次注册，超出容量返回 false。 */
bool record_reg(uint8_t serial_id, uint8_t frame_id) {
    if (g_reg_count >= MAX_REGS) {
        return false;
    }
    g_regs[g_reg_count].serial_id = serial_id;
    g_regs[g_reg_count].frame_id = frame_id;
    ++g_reg_count;
    return true;
}

/** 按记录逆序注销，忽略单个失败。 */
void rollback_regs() {
    auto &drv = usb_cdc_drv_t::instance();
    while (g_reg_count > 0) {
        --g_reg_count;
        (void)drv.remove_rx(g_regs[g_reg_count].serial_id,
                            g_regs[g_reg_count].frame_id);
    }
}

/**
 * @brief 回显 handler：从哪个口进来就从哪个口出去。
 *
 * 运行在 usb_task 上下文（tud_task_ext → on_cdc_rx → process_rx → dispatch），
 * 因此这里**只允许**做非阻塞的 send()。send() 内部 _tx_mutex 超时 1 tick，
 * 拿不到锁返回 PYRO_BUSY，此时本帧被丢弃——这是有意的背压策略：
 * 宁可丢一帧测试数据，也不能让 USB 任务卡住。
 *
 * 注意 p 指向 serial_t::_rx_buf，只在本次调用内有效，不能保存。
 */
bool echo_handler(uint8_t serial_id, uint8_t frame_id, const uint8_t *p,
                  uint16_t len) {
    if (serial_id < SERIAL_COUNT) {
        ++g_echo_count[serial_id];
    }
    return usb_cdc_drv_t::instance().send(serial_id, frame_id, p, len) == PYRO_OK;
}

/**
 * @brief identify handler：应答本端口身份。
 *
 * 载荷内容被忽略，只回一个固定 12 B 结构。上位机用它确认
 * 「COM 口 ↔ serial_id」的映射，双串口最容易错的就是两个口顺序反了。
 *
 * 注意 rx_event_handler 的签名是 (frame_id, data, len)，**不含 serial_id**，
 * 所以 serial_id 必须由注册时的 lambda 捕获后传进来。
 */
bool identify_handler(uint8_t serial_id, uint8_t frame_id, const uint8_t *p,
                      uint16_t len) {
    (void)frame_id;
    (void)p;
    (void)len;

    identify_t rsp{};
    rsp.serial_id = serial_id;
    rsp.magic = IDENTIFY_MAGIC;
    rsp.reserved = 0;
    rsp.echo_count = (serial_id < SERIAL_COUNT) ? g_echo_count[serial_id] : 0;
    rsp.tick_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);

    return usb_cdc_drv_t::instance().send(serial_id, FRAME_ID_IDENTIFY,
                                          reinterpret_cast<const uint8_t *>(&rsp),
                                          sizeof(rsp)) == PYRO_OK;
}

} // namespace

status_t usb_cdc_test_enable() {
    if (g_enabled) {
        return PYRO_OK;
    }

    auto &drv = usb_cdc_drv_t::instance();

    // 注意：这里**不能**检查 drv.is_ready()。
    // init() 只是把 usb_task 跑起来，此时主机通常还没枚举完，tud_ready() 是 false；
    // 而 handler 表在 mount/umount 时都不会被清空（on_mount() 只清 _rx_tail），
    // 所以挂载前注册完全没问题，反而必须挂载前注册，否则上电瞬间的帧会丢。

    g_reg_count = 0;

    for (uint8_t sid = 0; sid < SERIAL_COUNT; ++sid) {
        // identify
        if (drv.register_rx(sid, FRAME_ID_IDENTIFY,
                            [sid](uint8_t frame_id, const uint8_t *p, uint16_t len) {
                                return identify_handler(sid, frame_id, p, len);
                            }) != PYRO_OK) {
            rollback_regs();
            return PYRO_PARAM_ERROR;
        }
        if (!record_reg(sid, FRAME_ID_IDENTIFY)) {
            rollback_regs();
            return PYRO_NO_MEMORY;
        }

        // echo 0x7C..0x7F
        for (uint8_t i = 0; i < FRAME_ID_ECHO_COUNT; ++i) {
            const uint8_t fid = static_cast<uint8_t>(FRAME_ID_ECHO_BASE + i);
            // 用 lambda 捕获 sid，让回显严格走原路
            if (drv.register_rx(sid, fid,
                                [sid](uint8_t frame_id, const uint8_t *p, uint16_t len) {
                                    return echo_handler(sid, frame_id, p, len);
                                }) != PYRO_OK) {
                rollback_regs();
                return PYRO_PARAM_ERROR;
            }
            if (!record_reg(sid, fid)) {
                rollback_regs();
                return PYRO_NO_MEMORY;
            }
        }
    }

    for (uint8_t i = 0; i < SERIAL_COUNT; ++i) {
        g_echo_count[i] = 0;
    }
    g_enabled = true;
    return PYRO_OK;
}

status_t usb_cdc_test_disable() {
    if (!g_enabled) {
        return PYRO_OK;
    }
    rollback_regs();
    g_enabled = false;
    return PYRO_OK;
}

bool usb_cdc_test_is_enabled() { return g_enabled; }

uint32_t usb_cdc_test_echo_count(uint8_t serial_id) {
    if (serial_id >= SERIAL_COUNT) {
        return 0;
    }
    return g_echo_count[serial_id];
}

} // namespace pyro

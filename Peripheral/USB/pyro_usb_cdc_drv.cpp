#include "pyro_usb_cdc_drv.h"

#include "pyro_crc.h"
#include "tusb.h"
#include "stm32h7xx_hal.h"
#include "pyro_usb_desc_config.h"

namespace pyro {

usb_cdc_drv_t::~usb_cdc_drv_t() {
    delete _task;
    _task = nullptr;
}

usb_cdc_drv_t& usb_cdc_drv_t::instance() {
    static usb_cdc_drv_t inst;
    return inst;
}

status_t usb_cdc_drv_t::init(const char *serial = "PYRO-UNK-DEV") {
    if (_init)
        return PYRO_ALREADY_INIT;

    // 序列号为必填身份：缺失/超长/非可打印 ASCII 一律拒绝，避免"静默使用默认值或截断"
    // 导致同一 PC 上多块板被识别为同一设备（抢同一个 COM 号），这是最难排查的失效形态。
    // 长度上限与描述符缓冲的关系见 pyro_usb_desc_config.h。
    if (!serial || serial[0] == '\0')
        return PYRO_PARAM_ERROR;

    const size_t len = strlen(serial);
    if (len > static_cast<size_t>(PYRO_USB_SERIAL_MAX_LEN))
        return PYRO_PARAM_ERROR;

    for (size_t i = 0; i < len; ++i)
        if (serial[i] < 0x20 || serial[i] > 0x7E)
            return PYRO_PARAM_ERROR;

    // 注入描述符：必须先于 tusb_init()（USB 任务在下方才被创建并启动）
    pyro_usb_desc_set_serial(serial);

    _task = new usb_task_t(this);
    if (!_task)
        return PYRO_NO_MEMORY;

    const status_t ret = _task->start();
    if (ret != PYRO_OK) {
        delete _task;
        _task = nullptr;
        return ret;
    }

    //重置驱动状态
    reset();

    _init = true;
    return PYRO_OK;
}

status_t usb_cdc_drv_t::reset() {
    _rx_handlers.clear();
    return PYRO_OK;
}

status_t usb_cdc_drv_t::enable_rx() {
    if (!_init)
        return PYRO_ERROR;

    _rx_enable = true;
    return PYRO_OK;
}

status_t usb_cdc_drv_t::disable_rx() {
    _rx_enable = false;
    return PYRO_OK;
}

status_t usb_cdc_drv_t::register_rx(uint8_t frame_id, const rx_event_handler &handler) {
    if (!handler)
        return PYRO_PARAM_ERROR;

    for (auto it = _rx_handlers.begin(); it != _rx_handlers.end(); ++it)
        if (frame_id == (*it).first)
            return PYRO_PARAM_ERROR;

    _rx_handlers.push_back({frame_id, handler});
    return PYRO_OK;
}

status_t usb_cdc_drv_t::remove_rx(uint8_t frame_id) {
    for (auto it = _rx_handlers.begin(); it != _rx_handlers.end(); ++it)
        if (frame_id == (*it).first) {
            _rx_handlers.erase(it);
            return PYRO_OK;
        }

    return PYRO_NOT_FOUND;
}

status_t usb_cdc_drv_t::send(uint8_t frame_id, const uint8_t *data, uint16_t len) {
    if (len > usb_cdc::MAX_DATA_LENGTH)
        return PYRO_PARAM_ERROR;

    if (data == nullptr)
        return PYRO_PARAM_ERROR;

    if (!tud_mounted())
        return PYRO_ERROR;

    scoped_mutex_t lock(_tx_mutex, pdMS_TO_TICKS(5));
    if (!lock.is_locked())
        return PYRO_BUSY;

    uint16_t frame_len = len + 7;

    if (_tx_tail + frame_len >= usb_cdc::TX_BUF_CAPACITY)
        return PYRO_BUSY;

    uint8_t *p = _tx_buf + _tx_tail;
    frame_header_t header;
    header.sof = usb_cdc::SOF;
    header.frame_id = frame_id;
    header.data_len = len;
    header.crc8 = 0;
    memcpy(p, &header, sizeof(header));
    append_crc8_check_sum(p, sizeof(header));

    if (len > 0)
        memcpy(p + sizeof(header), data, len);

    append_crc16_check_sum(p, frame_len);

    _tx_tail += frame_len;

    return PYRO_OK;
}

void usb_cdc_drv_t::drain_tx() {
    if (_tx_tail == 0)
        return;

    if (!tud_mounted()){
        scoped_mutex_t lock(_tx_mutex, pdMS_TO_TICKS(1));
        if (lock.is_locked())
            _tx_tail = 0;
        return;
    }

    scoped_mutex_t lock(_tx_mutex, pdMS_TO_TICKS(1));
    if (!lock.is_locked())
        return; // send() 正在写缓冲，下个周期再来

    uint32_t space = tud_cdc_n_write_available(_itf);
    if (space == 0)
        return; // FIFO 满，等主机取走

    uint16_t chunk = (_tx_tail < space) ? _tx_tail : uint16_t(space);

    uint32_t written_len = tud_cdc_n_write(_itf, _tx_buf, chunk);
    if (written_len == 0)
        return;

    // 显式调用，使消息及时发出。默认会凑够一个64B完整数据包才自动发起传输
    tud_cdc_n_write_flush(_itf);

    uint16_t remain_len = (uint16_t)(_tx_tail - written_len);
    if (remain_len > 0)
        memmove(_tx_buf, _tx_buf + written_len, remain_len);
    _tx_tail = remain_len;
}

void usb_cdc_drv_t::on_cdc_rx() {
    while (tud_cdc_n_available(_itf)) {
        uint16_t space = usb_cdc::RX_BUF_CAPACITY - _rx_tail;
        if (space == 0) {
            _rx_tail = 0;
            continue;
        }

        uint32_t count = tud_cdc_n_read(_itf, _rx_buf + _rx_tail, space);
        if (count == 0)
            break;

        _rx_tail += count;

        process_rx();
    }
}

void usb_cdc_drv_t::process_rx() {
    uint16_t pos = 0;
    while (pos < _rx_tail) {
        uint8_t *st = _rx_buf + pos;
        uint16_t rest = _rx_tail - pos;

        if (rest < usb_cdc::HEADER_SIZE)
            break;

        if (st[0] != usb_cdc::SOF) {
            pos++;
            continue;
        }

        frame_header_t header;
        memcpy(&header, st, sizeof(header));

        if (!verify_crc8_check_sum(st, usb_cdc::HEADER_SIZE)) {
            pos++;
            continue;
        }

        if (header.data_len > usb_cdc::MAX_DATA_LENGTH) {
            pos++;
            continue;
        }

        uint16_t frame_len = header.data_len + usb_cdc::HEADER_SIZE + 2;
        if (rest < frame_len)
            break;

        if (!verify_crc16_check_sum(st, frame_len)) {
            pos++;
            continue;
        }

        dispatch(header.frame_id, st + usb_cdc::HEADER_SIZE, header.data_len);
        pos += frame_len;
    }

    if (pos > 0) {
        uint16_t remain_len = (uint16_t)(_rx_tail - pos);
        if (remain_len > 0)
            memmove(_rx_buf, _rx_buf + pos, remain_len);
        _rx_tail = remain_len;
    }
}

void usb_cdc_drv_t::dispatch(uint8_t frame_id, const uint8_t *data, uint16_t len) {
    for (auto it = _rx_handlers.begin(); it != _rx_handlers.end(); ++it)
        if (it->first == frame_id) {
            it->second(data, len);
            break;
        }
}

void usb_cdc_drv_t::on_mount() {
    _rx_tail = 0;
}

void usb_cdc_drv_t::on_umount() {
    // 丢弃主机残留数据，并清空尚未发出的 TX 帧
    tud_cdc_n_read_flush(_itf);
    _rx_tail = 0;

    scoped_mutex_t lock(_tx_mutex, pdMS_TO_TICKS(1));
    if (lock.is_locked())
        _tx_tail = 0;
}

/* ============================== USB 栈任务=========================== */
status_t usb_cdc_drv_t::usb_task_t::init()
{
    // 说明：USB 的时钟源（HSI48 / PLL1Q / PLL3Q）、USB 电压检测器、OTG_HS 外设时钟、
    //       以及 NVIC 优先级与使能，全部由 CubeMX 生成并维护的 HAL_PCD_MspInit() 完成
    //       （在 CubeMX/Core/Src/usb_otg.c 的 USER CODE 区被显式调用）。
    //       这样时钟树变化（例如把 USB 时钟从 PLL1Q 换成 HSI48）时无需改本驱动。
    //       ⚠️ 切勿在此硬编码 RCC_USBCLKSOURCE_*，否则与 .ioc 不符会直接导致 USB 无法枚举。

    // DP/DM 引脚：CubeMX 的 gpio.c 未配置 USB 引脚，故在此配置。
    // 内置 FS PHY 固定使用 PA11(D-) / PA12(D+)，AF10。
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF10_OTG1_HS;   // H723: == GPIO_AF10_OTG2_HS (0x0A)
    HAL_GPIO_Init(GPIOA, &gpio);

    // 强制中断优先级满足 FreeRTOS 约束（必须 <= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）。
    // MspInit 里已设过一次（值为 5），这里再显式覆盖一次，以防后续 CubeMX 改动优先级。
    HAL_NVIC_SetPriority(OTG_HS_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);

    return PYRO_OK;
}

void usb_cdc_drv_t::usb_task_t::run_loop() {
    // 必须在调度器启动后调用（ISR 中使用了 RTOS 队列 API）
    const tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,     // 内置 FS PHY
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    while (true) {
        tud_task_ext(1, false);
        if (_owner != nullptr)
            _owner->drain_tx();
    }
}

}

/* ============================ C 桥接层 ================================ */
extern "C" {

void tud_cdc_rx_cb(uint8_t itf) {
    //目前写死单端口，之后拓展时可修改
    if (itf == 0)
        pyro::usb_cdc_drv_t::instance().on_cdc_rx();
}

void tud_mount_cb(void) {
    pyro::usb_cdc_drv_t::instance().on_mount();
}

void tud_umount_cb(void) {
    pyro::usb_cdc_drv_t::instance().on_umount();
}

/**
 * @brief 供 CubeMX 生成的 OTG_HS_IRQHandler 转发的桥接函数。
 * @note 中断向量的定义保留在 CubeMX/Core/Src/stm32h7xx_it.c 中（其 USER CODE 区调用本函数），
 *       这样 CubeMX 重新生成代码不会造成符号丢失或重复。
 *       编号必须与 tusb_init()/BOARD_TUD_RHPORT 一致。
 */
void pyro_usb_irq_handler(void) {
    tusb_int_handler(BOARD_TUD_RHPORT, true);
}

}
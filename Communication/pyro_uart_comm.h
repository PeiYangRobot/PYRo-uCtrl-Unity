/**
 * @file pyro_uart_comm.h
 * @brief UART 通信应用层二次封装
 * * 核心特性：
 * 1. 泛型收发：直接传入结构体/基础类型进行 read/write，自动计算 sizeof。
 * 2. 消息队列：底层基于 FreeRTOS MessageBuffer，解耦 ISR 与 Task。
 * 3. 智能解析：在 DMA
 * 空闲中断中，采用滑动窗口扫描机制自动剥离合法帧，免疫粘包与错位乱码。
 * 4. 线程安全：内置自定义 mutex_t 锁，支持多任务高并发安全读写。
 * * @author
 * @date
 */

#ifndef __PYRO_UART_COMM_H__
#define __PYRO_UART_COMM_H__

#include "pyro_uart_drv.h"
#include "pyro_mutex.h"
#include "message_buffer.h"
#include <vector>
#include <cstring>

namespace pyro
{

/**
 * @brief 消息注册配置结构体，用于在中断中匹配合法数据包
 */
struct msg_config_t
{
    uint16_t length;    // 消息体总长度
    uint8_t header[8];  // 帧头数据 (最大支持 8 字节)
    uint8_t header_len; // 帧头实际长度，0 代表无帧头校验
};

class uart_comm_t
{
  public:
    /**
     * @brief 构造函数
     * @param uart_drv      选择的 UART 外设
     * @param owner_id     回调的拥有者 ID，用于底层区分不同应用
     * @param msg_buf_size FreeRTOS 消息缓冲区的总大小（字节数）
     */
    uart_comm_t(const uart_drv_t& uart_drv, uint32_t owner_id,
                size_t msg_buf_size = 1024);

    ~uart_comm_t();

    // 禁用拷贝构造和赋值操作，防止资源重复释放
    uart_comm_t(const uart_comm_t &)            = delete;
    uart_comm_t &operator=(const uart_comm_t &) = delete;

    /* ===================================================================== */
    /* 消息过滤注册接口 (底层基础类型重载)                                */
    /* ===================================================================== */

    /**
     * @brief 注册允许接收的消息类型 (仅指定长度，无帧头校验)
     * @param length 消息总长度
     */
    void register_msg_type(uint16_t length);

    /**
     * @brief 注册允许接收的消息类型 (指定长度与帧头)
     * @param length     消息总长度
     * @param header     帧头数组指针
     * @param header_len 帧头长度
     */
    void register_msg_type(uint16_t length, const uint8_t *header,
                           uint8_t header_len);

    /* ===================================================================== */
    /* 消息过滤注册接口 (泛型模板重载)                                   */
    /* ===================================================================== */

    /**
     * @brief 泛型注册 (仅指定类型，自动推导长度，无帧头校验)
     * @tparam T 消息结构体或基本数据类型
     */
    template <typename T> void register_msg_type()
    {
        register_msg_type(sizeof(T));
    }

    /**
     * @brief 泛型注册 (指定类型与帧头，自动推导长度)
     * @tparam T 消息结构体或基本数据类型
     * @param header     帧头数组指针
     * @param header_len 帧头长度
     */
    template <typename T>
    void register_msg_type(const uint8_t *header, uint8_t header_len)
    {
        register_msg_type(sizeof(T), header, header_len);
    }

    /* ===================================================================== */
    /* 泛型收发接口                                   */
    /* ===================================================================== */

    /**
     * @brief 泛型非阻塞发送 (DMA)
     * @tparam T 消息结构体或基本数据类型
     * @param packet 要发送的数据包引用
     * @return true 发送成功, false 失败
     */
    template <typename T> bool write(const T &packet)
    {
        scoped_mutex_t lock(_tx_mutex);
        return (_drv.write(reinterpret_cast<const uint8_t *>(&packet),
                            sizeof(T)) == PYRO_OK);
    }

    /**
     * @brief 泛型阻塞发送 (Polling)
     * @tparam T 消息结构体或基本数据类型
     * @param packet 要发送的数据包引用
     * @param timeout_ticks 发送超时时间 (FreeRTOS Ticks)
     * @return true 发送成功, false 超时或失败
     */
    template <typename T>
    bool write_blocking(const T &packet, TickType_t timeout_ticks)
    {
        scoped_mutex_t lock(_tx_mutex, timeout_ticks);
        if (!lock.is_locked())
            return false;
        return (_drv.write(reinterpret_cast<const uint8_t *>(&packet),
                            sizeof(T), timeout_ticks) == PYRO_OK);
    }

    /**
     * @brief 从消息队列中读取一帧完整数据
     * @tparam T 消息结构体或基本数据类型
     * @param out_packet 用于接收数据的结构体引用
     * @param timeout_ticks 阻塞等待时间 (0 为非阻塞立刻返回)
     * @return true 成功读出一帧匹配 T 长度的数据, false 缓冲区空或超时
     */
    template <typename T> bool read(T &out_packet, TickType_t timeout_ticks = 0)
    {
        // 加读锁防止多任务同时提取 MessageBuffer 导致数据包被拆散分发
        scoped_mutex_t lock(_rx_mutex);

        size_t rx_len = xMessageBufferReceive(_msg_buffer, &out_packet,
                                              sizeof(T), timeout_ticks);

        return (rx_len == sizeof(T));
    }

  private:
    uart_drv_t _drv;   // 底层 UART 驱动单例指针
    uint32_t _owner_id; // 当前通信实例的独立 ID

    mutex_t _tx_mutex; // 发送接口线程安全锁
    mutex_t _rx_mutex; // 接收接口线程安全锁

    MessageBufferHandle_t _msg_buffer;      // FreeRTOS 消息缓冲区句柄
    std::vector<msg_config_t> _msg_configs; // 注册的消息类型白名单规则表

    /**
     * @brief 底层中断回调，用于滑动窗口解析数据包
     */
    bool internal_rx_callback(uint8_t *p, uint16_t size,
                              BaseType_t &xHigherPriorityTaskWoken);
};

} // namespace pyro

#endif // __PYRO_UART_COMM_H__
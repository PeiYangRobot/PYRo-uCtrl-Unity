# PYRo RC Driver

**基于 FreeRTOS 与 PYRo UART Driver 的多协议遥控器驱动框架**

该模块 (`pyro_rc_drv`) 采用工厂模式统一管理多种遥控协议（DR16, VT03，...）。它利用 FreeRTOS 消息缓冲区实现软硬解耦，通过全局位掩码实现多接收机优先级的动态仲裁，并提供基于时间戳的事件管理机制。

---

## Part 1: 代码全解 (Code Deep Dive)

### 1. 实例构建：工厂与隐式初始化

系统通过 `rc_hub_t` 统一管理实例，用户无需手动管理内存或调用初始化函数。

#### 1.1 懒加载与自动绑定

在 `pyro_rc_hub.cpp` 中，利用 C++ 局部静态变量特性，实现了按需加载。

```C++
/* pyro_rc_hub.cpp */
rc_drv_t *rc_hub_t::get_instance(which_rc_t which_rc)
{
    // ...
    // 1. 自动获取底层 UART 驱动 (硬件绑定)
    static uart_drv_t *vt03_uart = uart_drv_t::get_instance(...);
    // 2. 构造驱动实例 (仅首次调用时执行，自动调用 init)
    static vt03_drv_t vt03_rc_drv(vt03_uart);
    return &vt03_rc_drv;
}
```

- **解读**：只有当首次调用 `get_instance` 时，驱动对象才会被创建。构造函数内部会自动申请 FreeRTOS 任务与缓冲区资源，因此获取即就绪。
    

### 2. 连接控制：物理链路的挂载与切断

`enable` 和 `disable` 并非简单的标志位切换，而是直接操作底层 UART 的中断回调链表。

#### 2.1 建立连接 (`enable`)

```C++
/* pyro_vt03_rc_drv.cpp */
void vt03_drv_t::enable() {
    // 向 UART 驱动注册 ISR 回调，物理接通数据流
    _rc_uart->add_rx_event_callback(
        [this](...) { return rc_callback(...); }, 
        reinterpret_cast<uint32_t>(this)
    );
}
```

- **实质**：将驱动的接收函数挂载到uart驱动回调向量表中。此前 UART 中断会忽略该数据，接收函数用遥控器的帧特征（如长度/帧头等）检测硬件收到的消息是否是遥控器的数据，如不是，则返回false，是则返回bool（此部分具体参考串口文档）

#### 2.2 断开连接 (`disable`)

```C++
void vt03_drv_t::disable() {
    // 1. 立即清除在线状态位 (原子操作) -> 释放仲裁锁
    sequence &= ~(1 << _priority);
    // 2. 从 UART 驱动移除回调 -> 物理断开
    _rc_uart->remove_rx_event_callback(...);
}
```

- **实质**：除了停止接收，最关键的是**立即释放优先权**。若此时有低优先级设备（如 DR16）在线，它将即刻接管控制权。
    

### 3. 中断仲裁：抢占式过滤

在 UART 中断回调 (`rc_callback`) 中，驱动执行核心的优先级仲裁。

```C++
/* pyro_vt03_rc_drv.cpp */
bool vt03_drv_t::rc_callback(...) {
    // __builtin_ctz(sequence): 获取当前活跃的最高优先级
    // 规则：仅当 本驱动优先级 <= 当前系统最高优先级 时，才允许通过
    if (__builtin_ctz(sequence) >= _priority) {
        xMessageBufferSendFromISR(...); // 推入缓冲区
        return true;
    }
    return false;
}
```

- **解读**：若高优先级设备（VT03, Prio 0）在线，低优先级设备（DR16, Prio 1）的数据会在中断层被直接丢弃，不消耗任务层资源。
    

### 4. 线程逻辑：离线与在线的自动切换

处理任务 (`thread`) 采用状态机机制，适配遥控器 **14ms** 的发送周期。

#### 4.1 阶段一：离线等待 (Wait)

```C++
/* pyro_vt03_rc_drv.cpp */
// 使用 portMAX_DELAY 永久阻塞，不消耗 CPU
if (xMessageBufferReceive(..., portMAX_DELAY) == sizeof(vt03_buf_t)) {
    // 收到第一帧数据 -> 标记上线
    sequence |= (1 << _priority);
}
```

- **机制**：任务默认挂起。一旦收到首帧数据，立即唤醒并将 `sequence` 对应位置 1，宣告设备上线。
    
#### 4.2 阶段二：在线保活 (Active Loop)

```C++
while (sequence >> _priority & 0x01) {
    // 超时接收：120ms (约允许丢包 8 帧: 14ms * 8 = 112ms)
    xReceivedBytes = xMessageBufferReceive(..., 120);
    
    if (xReceivedBytes == 0) {
        // 超时 -> 判定断连 -> 清除在线位 -> 回到阶段一
        sequence &= ~(1 << _priority);
    }
}
```

- **机制**：若 120ms 内无新数据，驱动认定物理连接断开，自动清除在线状态，释放控制权。

### 5. 事件管理：时间戳机制 (`change_time`)

为解决“按键一次触发多次”的问题，驱动将“状态”与“事件”解耦。在使用代表事件的量（如拨杆切换操作，按键按下操作等）时应用时间戳去判断是否是新包。

```C++
/* pyro_vt03_rc_drv.cpp :: check_ctrl */
// 当检测到状态变化（如 按下、松开、长按触发）时
key.change_time = now; // 记录事件发生的时间戳
```

- **应用**：业务层只需判断 `key.change_time > last_processed_time` 即可识别这是否是一个新的操作指令。

## Part 2: 快速上手 (Quick Start)

### 1. 启动驱动

无需手动 `init`，直接获取实例并使能。

```C++
#include "pyro_rc_hub.h"

void robot_init() {
    // 获取实例并建立物理连接
    pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->enable();
}
```

### 2. 读取数据与事件处理 (最佳实践)

下面的代码展示了如何利用 `change_time` 处理按键逻辑，确保**按一次只触发一次**。

```C++
#include "pyro_vt03_rc_drv.h"
#include "pyro_dwt_drv.h" // 用于获取当前时间或其他时间相关操作

void control_task() {
    auto *rc_drv = pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03);
    
    // 【关键】本地变量，记录上一次处理按键的时间戳
    static float last_process_time = 0.0f;

    while (1) {
        if (rc_drv->check_online()) {
            pyro::read_scope_lock lock(rc_drv->get_lock());
            const auto *data = 
	            static_cast<const pyro::vt03_drv_t::vt03_ctrl_t *>(rc_drv->read());

            // --- 1. 处理连续量 (摇杆) ---
            // 直接使用，无需时间戳判断
            float speed = data->rc.ch_ly; 

            // --- 2. 处理离散事件 (按键/拨杆) ---
            // 检查左功能键是否有“新”的动作发生
            if (data->rc.fn_l.change_time > last_process_time)
            {
                // 这是一个新事件！判断具体类型：
                if (data->rc.fn_l.ctrl == pyro::vt03_drv_t::key_ctrl_t::KEY_PRESSED) {
                    // 处理单击：切换模式
                    toggle_mode();
                }
                else if (data->rc.fn_l.ctrl == pyro::vt03_drv_t::key_ctrl_t::KEY_HOLD) {
                    // 处理长按：强制复位
                    system_reset();
                }
                
                // 【重要】更新本地时间戳，确保同一个事件不会被处理第二次
                last_process_time = data->rc.fn_l.change_time;
            }
        }
        else {
            robot_stop();
        }
        vTaskDelay(2);
    }
}
```

> tip：对于按键，多次连击的最后一下长按被认为是**带有连击属性**的长按操作

### 3. 停止连接

当需要停止连接时，调用 `disable` 会立即切断信号流。

```C++
// 1. 物理层断开 UART 回调
// 2. 逻辑层立即清除 Online 标志
// 3. 此时若有低优先级遥控器在线，它将立即接管控制权
pyro::rc_hub_t::get_instance(pyro::rc_hub_t::VT03)->disable();
```
/**
 * @file pyro_vl53_reg.h
 * @brief VL53-100 激光测距传感器硬件寄存器与参数定义
 */

#ifndef __PYRO_VL53_REG_H__
#define __PYRO_VL53_REG_H__

/* ------------------ Modbus 功能码 ------------------ */
#define VL53_FUNC_READ_REG              0x03    // 读寄存器
#define VL53_FUNC_WRITE_REG             0x06    // 写单个寄存器

/* ------------------ 寄存器地址 ------------------ */
#define VL53_REG_SYS_RESET              0x00    // 系统恢复
#define VL53_REG_BAUD_RATE              0x03    // 波特率 / 回传速度设置
#define VL53_REG_DEVICE_ID              0x1A    // 设备 ID 设置
#define VL53_REG_DISTANCE               0x34    // 测量数据 (只读)
#define VL53_REG_OUT_STATUS             0x35    // 输出状态 (只读)
#define VL53_REG_MEASURE_MODE           0x36    // 测量模式
#define VL53_REG_CALIBRATION            0x37    // 校准模式
#define VL53_REG_SYSTEM_MODE            0x38    // 系统工作模式

/* ------------------ 寄存器配置值枚举 ------------------ */

// 1. 波特率设置 (写 0x03 寄存器)
#define VL53_BAUD_2400                  0x00
#define VL53_BAUD_4800                  0x01
#define VL53_BAUD_9600                  0x02
#define VL53_BAUD_19200                 0x03
#define VL53_BAUD_38400                 0x04
#define VL53_BAUD_57600                 0x05
#define VL53_BAUD_115200                0x06    // 出厂默认
#define VL53_BAUD_230400                0x07
#define VL53_BAUD_460800                0x08
#define VL53_BAUD_921600                0x09

// 2. 自动回传速率 (写 0x03 寄存器，使用功能码 0x06 时高位表示速率类型)
// 注意：数据手册中波特率和回传速率共用 0x03，此处仅列出常用回传速率值
#define VL53_RATE_0_1_HZ                0x01
#define VL53_RATE_0_5_HZ                0x02
#define VL53_RATE_1_HZ                  0x03
#define VL53_RATE_2_HZ                  0x04
#define VL53_RATE_5_HZ                  0x05
#define VL53_RATE_10_HZ                 0x06    // 出厂默认

// 3. 测量模式 (写 0x36 寄存器)
#define VL53_MEAS_MODE_NORMAL           0x00    // 一般模式 (30ms, 1.2m) - 默认
#define VL53_MEAS_MODE_HIGH_PRECISION   0x01    // 高精度模式 (200ms, 1.2m, 误差±3%)
#define VL53_MEAS_MODE_LONG_DIST        0x02    // 长距离模式 (33ms, 2m)
#define VL53_MEAS_MODE_HIGH_SPEED       0x03    // 高速模式 (20ms, 1.2m, 误差±5%)

// 4. 系统工作模式 (写 0x38 寄存器)
#define VL53_SYS_MODE_AUTO_NORMAL       0x00    // 正常模式 (自动回传 ASCII 字符串) - 默认
#define VL53_SYS_MODE_MODBUS            0x01    // Modbus 模式 (一问一答，十六进制)
#define VL53_SYS_MODE_IIC               0x02    // IIC 模式

// 5. 输出状态指示位 (读取数据时返回的状态)
#define VL53_STATE_VALID                0x00    // 测量范围内 (Range Valid)
#define VL53_STATE_SIGMA_FAIL           0x01    // Sigma 错误
#define VL53_STATE_SIGNAL_FAIL          0x02    // 信号错误
#define VL53_STATE_MIN_RANGE_FAIL       0x03    // 小于最小测量范围
#define VL53_STATE_PHASE_FAIL           0x04    // 超出测量范围
#define VL53_STATE_HARDWARE_FAIL        0x05    // 硬件错误
#define VL53_STATE_NO_UPDATE            0xFF    // 无数据更新

// 6. 其他特殊操作指令
#define VL53_CMD_RESTORE_FACTORY        0x01    // 写入 0x00 寄存器恢复出厂
#define VL53_CMD_START_CALIBRATION      0x04    // 写入 0x37 寄存器进入校准

#endif // __PYRO_VL53_REG_H__
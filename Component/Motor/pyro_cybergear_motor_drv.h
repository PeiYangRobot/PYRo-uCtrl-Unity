/**
 * @version 1.1.0
 */

#ifndef CYBERGEAR_MOTOR_DRV_H
#define CYBERGEAR_MOTOR_DRV_H

#include "pyro_motor_base.h"
#include <cstdint>

namespace pyro
{

/**
 * CyberGear电机CAN扩展帧ID结构:
 * bit[28:24] - 通信类型 (5位)
 * bit[23:8]  - 数据区2 (16位)
 * bit[7:0]   - 目标地址/电机ID (8位)
 */
class cybergear_can_id_t
{
  public:
    cybergear_can_id_t(uint8_t comm_type, uint16_t data2, uint8_t motor_id);
    cybergear_can_id_t(uint32_t raw_extended_id);

    // 将三个字段组合成29位CAN扩展帧ID (发送时使用)
    uint32_t encode() const;
    // 从29位CAN扩展帧ID中提取三个字段 (接收时使用)
    void decode(uint32_t raw_extended_id);

    uint8_t get_comm_type() const { return _comm_type; }
    uint16_t get_data2() const { return _data2; }
    uint8_t get_motor_id() const { return _motor_id; }

    void set_comm_type(uint8_t type) { _comm_type = type; }
    void set_data2(uint16_t data2) { _data2 = data2; }
    void set_motor_id(uint8_t id) { _motor_id = id; }

  private:
    uint8_t _comm_type;  // bit[28:24]
    uint16_t _data2;     // bit[23:8]
    uint8_t _motor_id;   // bit[7:0]

    static constexpr uint32_t COMM_TYPE_MASK = 0x1F000000;
    static constexpr uint32_t DATA2_MASK = 0x00FFFF00;
    static constexpr uint32_t MOTOR_ID_MASK = 0x000000FF;
    static constexpr uint8_t COMM_TYPE_SHIFT = 24;
    static constexpr uint8_t DATA2_SHIFT = 8;
};

class cybergear_motor_drv_t : public motor_base_t
{
  public:
    cybergear_motor_drv_t(uint8_t motor_id, uint8_t master_id, bsp_can::which_can which);
    ~cybergear_motor_drv_t();

    status_t enable() override;
    status_t disable() override;
    status_t clear_fault();

    // 设置电机机械零位（将当前位置设为零位，掉电丢失）
    status_t set_mechanical_zero();

    // 设置电机CAN_ID（立即生效，应答帧格式同get_device_id）
    status_t set_can_id(uint8_t new_can_id);

    // 参数索引枚举
    enum param_index_t : uint16_t
    {
        PARAM_RUN_MODE = 0x7005,        // 运行模式 (uint8)
        PARAM_IQ_REF = 0x7006,          // 电流模式Iq指令 (float, -23~23A)
        PARAM_SPD_REF = 0x700A,         // 转速模式转速指令 (float, -30~30rad/s)
        PARAM_LIMIT_TORQUE = 0x700B,    // 转矩限制 (float, 0~12Nm)
        PARAM_CUR_KP = 0x7010,          // 电流Kp (float, 默认0.125)
        PARAM_CUR_KI = 0x7011,          // 电流Ki (float, 默认0.0158)
        PARAM_CUR_FILT_GAIN = 0x7014,   // 电流滤波系数 (float, 0~1.0, 默认0.1)
        PARAM_LOC_REF = 0x7016,         // 位置模式角度指令 (float, rad)
        PARAM_LIMIT_SPD = 0x7017,       // 位置模式速度限制 (float, 0~30rad/s)
        PARAM_LIMIT_CUR = 0x7018,       // 速度位置模式电流限制 (float, 0~23A)
        PARAM_MECH_POS = 0x7019,        // 负载端计圈机械角度 (float, rad)
        PARAM_IQF = 0x701A,             // iq滤波值 (float, -23~23A)
        PARAM_MECH_VEL = 0x701B,        // 负载端转速 (float, -30~30rad/s)
        PARAM_VBUS = 0x701C,            // 母线电压 (float, V)
        PARAM_ROTATION = 0x701D,        // 圈数 (int16)
        PARAM_LOC_KP = 0x701E,          // 位置Kp (float, 默认30)
        PARAM_SPD_KP = 0x701F,          // 速度Kp (float, 默认1)
        PARAM_SPD_KI = 0x7020,          // 速度Ki (float, 默认0.002)
    };

    // 读取单个参数
    status_t read_param(param_index_t index, float &value);
    status_t read_param(param_index_t index, uint8_t &value);
    status_t read_param(param_index_t index, int16_t &value);

    // 写入单个参数（掉电丢失）
    status_t write_param(param_index_t index, float value);
    status_t write_param(param_index_t index, uint8_t value);
    status_t write_param(param_index_t index, int16_t value);

    status_t update_feedback() override;
    status_t send_torque(float torque) override;

    // 获取电机设备ID和MCU唯一标识符
    status_t get_device_id(uint64_t &mcu_id);

    // 获取电机故障状态
    struct fault_status_t
    {
        bool uncalibrated;    // bit21: 未标定
        bool hall_error;      // bit20: HALL编码故障
        bool magnetic_error;  // bit19: 磁编码故障
        bool over_temp;       // bit18: 过温
        bool over_current;    // bit17: 过流
        bool under_voltage;   // bit16: 欠压故障
    };

    enum motor_mode_t : uint8_t
    {
        MODE_RESET = 0,  // Reset模式[复位]
        MODE_CALI = 1,   // Cali模式[标定]
        MODE_MOTOR = 2   // Motor模式[运行]
    };

    fault_status_t get_fault_status() const { return _fault_status; }
    motor_mode_t get_motor_mode() const { return _motor_mode; }

    // 详细故障信息（来自故障反馈帧）
    struct detailed_fault_t
    {
        // fault值 (Byte0~3)
        bool phase_a_over_current;      // bit16: A相电流采样过流
        bool overload;                  // bit15~8: 过载故障
        bool encoder_uncalibrated;      // bit7: 编码器未标定
        bool phase_c_over_current;      // bit5: C相电流采样过流
        bool phase_b_over_current;      // bit4: B相电流采样过流
        bool over_voltage;              // bit3: 过压故障
        bool under_voltage;             // bit2: 欠压故障
        bool driver_chip_fault;         // bit1: 驱动芯片故障
        bool motor_over_temp;           // bit0: 电机过温故障，默认80度

        // warning值 (Byte4~7)
        bool motor_temp_warning;        // bit0: 电机过温预警，默认75度
    };

    detailed_fault_t get_detailed_fault() const { return _detailed_fault; }

    // 运行模式枚举（对应PARAM_RUN_MODE参数）
    enum run_mode_t : uint8_t
    {
        MODE_MOTION_CONTROL = 0,  // 运控模式（默认）
        MODE_POSITION = 1,        // 位置模式
        MODE_SPEED = 2,           // 速度模式
        MODE_CURRENT = 3,         // 电流模式
    };

    // 模式切换和控制的高层接口
    status_t set_run_mode(run_mode_t mode);

    // 模式专用控制接口（推荐使用）
    status_t send_control(float value);                      // 单变量控制（根据当前模式自动分配）
    status_t send_control(float value, float kp, float kd);  // 位置控制（带自定义PID参数）

    // 运控模式控制指令（通用接口，需要根据模式填入对应字段）
    status_t send_motion_control(float target_angle, float target_velocity, float kp, float kd,
                                 float torque);

    // 模式运行中的参数更新（仅用于配置，不是实时控制）
    status_t set_current_ref(float iq_ref);
    status_t set_speed_ref(float spd_ref);
    status_t set_position_ref(float loc_ref);

    // 设置反馈数据的物理范围（用于正确解析反馈）
    void set_position_range(float min, float max) override;
    void set_rotate_range(float min, float max) override;
    void set_torque_range(float min, float max) override;

  protected:
    uint8_t _motor_id;    // 目标电机ID
    uint8_t _master_id;   // 主机ID

    fault_status_t _fault_status; // 故障状态
    motor_mode_t _motor_mode;     // 电机模式（从反馈中解析）
    detailed_fault_t _detailed_fault; // 详细故障信息

    run_mode_t _current_run_mode; // 当前设置的运行模式（用于send_control自动分配）

    // 反馈数据的物理范围（用于解析）
    float _min_position;
    float _max_position;
    float _min_velocity;
    float _max_velocity;
    float _min_torque;
    float _max_torque;

    can_msg_buffer_t *_device_id_msg;       // 设备ID应答消息缓冲区
    can_msg_buffer_t *_motion_feedback_msg; // 运控模式反馈消息缓冲区
    can_msg_buffer_t *_param_response_msg;  // 参数读写应答消息缓冲区
    can_msg_buffer_t *_fault_feedback_msg;  // 故障反馈消息缓冲区
};

} // namespace pyro

#endif

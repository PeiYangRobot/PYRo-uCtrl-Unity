#ifndef __PYRO_CHASSIS_BASE_H__
#define __PYRO_CHASSIS_BASE_H__

#include "FreeRTOS.h"
#include "pyro_mutex.h"
#include "task.h"


namespace pyro
{
class chassis_base_t
{
  public:
    /**
     * @brief Chassis Type Enumeration
     * Defined inside the class for scope protection.
     */
    enum class type_t
    {
        UNKNOWN,
        MECANUM,   // Mecanum wheel chassis
        WHEEL_LEG, // Wheel-legged robot
        OMNI,      // Omni wheel chassis
        RUDDER     // Rudder wheel (Swerve) chassis
    };

    /**
     * @brief Base Control Packet (The "Protocol")
     * Nested structure defining the common interface data.
     */
    struct cmd_base_t
    {
        type_t type;        // Type tag for runtime safety checks
        uint32_t timestamp; // Timestamp for timeout detection (ms/tick)

        float vx; // Linear velocity X (m/s)
        float vy; // Linear velocity Y (m/s)
        float wz; // Angular velocity Z (rad/s)
        // Yaw error for heading control (rad)
        // if not followed gimbal yaw, set to 0


        /**
         * @brief Constructor with default initialization
         * @param t Specific chassis type
         */
        explicit cmd_base_t(const type_t t = type_t::UNKNOWN)
            : type(t), timestamp(0), vx(0.0f), vy(0.0f), wz(0.0f)
        {
        }
        // Ensure virtual destructor for polymorphism
        virtual ~cmd_base_t() = default;
    };
    virtual void init()                             = 0;
    virtual void set_command(const cmd_base_t &cmd) = 0;
    void thread();
    virtual ~chassis_base_t() = default;
    /**
     * @brief Get the chassis type
     * @return type_e Current chassis type
     */
    [[nodiscard]] type_t get_type() const
    {
        return _type;
    }

    [[nodiscard]] mutex_t &get_mutex()
    {
        return _mutex;
    }

    TaskHandle_t _chassis_task_handle{};

  protected:
    virtual void update_feedback()                  = 0;
    virtual void kinematics_solve()                 = 0;
    virtual void chassis_control()                  = 0;
    virtual void power_control()                    = 0;
    virtual void send_motor_command()               = 0;
    mutex_t _mutex;
    TaskHandle_t _chassis_init_handle{};
    chassis_base_t();
    explicit chassis_base_t(type_t type);


    // Protected member variables (prefixed with _)
    type_t _type{};
};
} // namespace pyro
#endif

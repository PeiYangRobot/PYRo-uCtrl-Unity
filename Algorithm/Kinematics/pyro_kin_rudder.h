#ifndef __PYRO_KIN_RUDDER__
#define __PYRO_KIN_RUDDER__

namespace pyro
{

/**
 * @brief Rudder (Steerable Wheel) kinematics solver class
 */
class rudder_kin_t
{
  public:
    // Index enum for readable array access
    enum wheel_index_e
    {
        FL = 0, // Front Left
        FR = 1, // Front Right
        BL = 2, // Back Left
        BR = 3  // Back Right
    };

    struct module_state_t
    {
        float speed; // Linear speed (m/s)
        float angle; // Steering angle (rad, -PI to +PI)
        int direction; // Wheel direction
    };

    struct rudder_states_t
    {
        module_state_t modules[4]; // Array storing state for all 4 modules
    };

    /**
     * @brief Constructor
     * @param wheelbase   Wheelbase (Distance between front and rear axles, m)
     * @param track_width Track width (Distance between left and right wheels, m)
     */
    rudder_kin_t(float wheelbase, float track_width);

    /**
     * @brief Inverse Kinematics (Body Velocity -> Module States)
     * Calculates target speed and angle for each module with smart optimization.
     * @param vx  Linear velocity in X-axis (Forward +, m/s)
     * @param vy  Linear velocity in Y-axis (Left +, m/s)
     * @param wz  Angular velocity in Z-axis (Counter-Clockwise +, rad/s)
     * @param current_states Current states of modules (Required for shortest path optimization)
     * @return rudder_states_t Target state for each module
     */
    [[nodiscard]] rudder_states_t solve(float vx, float vy, float wz,
                                        const rudder_states_t &current_states) const;

  private:
    // Geometry half-lengths
    // Private variables start with _
    float _half_wheelbase;
    float _half_track_width;

    // Speed deadband to prevent steering jitter when stationary
    const float _deadband = 1e-3f;

    /**
     * @brief Helper to perform "Smart Selection" (Closest Angle & Reverse)
     */
    // void _optimize_module(float target_vx, float target_vy,
    //                       const module_state_t &current_state,
    //                       module_state_t &out_state) const;

    static void calc_angle(float &target_angle, float current_angle, int &direction);

    // Utility: Normalize angle to [-PI, PI]
    static float _normalize_angle(float angle);
};

} // namespace pyro

#endif
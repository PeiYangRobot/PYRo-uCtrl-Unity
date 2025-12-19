#ifndef __PYRO_KIN_MEC__
#define __PYRO_KIN_MEC__

namespace pyro
{

/**
 * @brief Mecanum kinematics solver class
 */
class mecanum_kin_t
{
  public:
    struct wheel_speeds_t
    {
        float fl; // Front Left
        float fr; // Front Right
        float bl; // Back Left
        float br; // Back Right
    };

    /**
     * @brief Constructor
     * @param wheelbase   Wheelbase (Distance between front and rear axles, m)
     * @param track_width Track width (Distance between left and right wheels,
     * m)
     */
    mecanum_kin_t(float wheelbase, float track_width);

    /**
     * @brief Inverse Kinematics (Body Velocity -> Wheel Speeds)
     * Calculates target wheel speeds from robot target velocity.
     * @param vx  Linear velocity in X-axis (Forward +, m/s)
     * @param vy  Linear velocity in Y-axis (Left +, m/s)
     * @param wz  Angular velocity in Z-axis (Counter-Clockwise +, rad/s)
     * @return wheel_speeds_t Target linear speed for each wheel (m/s)
     */
    [[nodiscard]] wheel_speeds_t solve(float vx, float vy, float wz) const;

    /**
     * @brief Forward Kinematics (Wheel Speeds -> Body Velocity)
     * Estimates robot velocity from actual wheel speeds (Used for Odometry).
     * @param speeds Current linear speed of each wheel (m/s)
     * @param out_vx Output reference: Body velocity X
     * @param out_vy Output reference: Body velocity Y
     * @param out_wz Output reference: Body angular velocity Z
     */
    void compute_odometry(const wheel_speeds_t &speeds, float &out_vx,
                          float &out_vy, float &out_wz) const;

  private:
    // Geometry calculation constant: (lx + ly)
    // Private variables start with _
    float _k_geom;
};

} // namespace pyro

#endif
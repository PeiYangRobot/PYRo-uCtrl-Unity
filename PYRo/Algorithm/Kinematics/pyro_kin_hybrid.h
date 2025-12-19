#ifndef __PYRO_KIN_HYBRID__
#define __PYRO_KIN_HYBRID__

namespace pyro
{

/**
 * @brief Hybrid Chassis Kinematics (Tracks + Mecanum)
 */
class hybrid_kin_t
{
public:
    /**
     * @brief Operation modes for the hybrid chassis
     */
    enum class drive_mode_t
    {
        CRUISING, // Only Mecanum active. Tracks = 0. Allows Strafing (Vy).
        CLIMBING  // Tracks + Mecanum active. No Strafing (Vy forced to 0).
    };

    /**
     * @brief Output structure containing target speeds for all 6 traction motors
     */
    struct hybrid_speeds_t
    {
        // Front Tracks
        float track_l;
        float track_r;

        // Rear Mecanum Wheels
        float mec_fl;
        float mec_fr;
        float mec_bl;
        float mec_br;
    };

    /**
     * @brief Constructor
     * @param track_spacing    Distance between the centers of the two tracks (m)
     * @param mec_wheelbase    Wheelbase of the mecanum section (m)
     * @param mec_track_width  Track width of the mecanum section (m)
     */
    hybrid_kin_t(float track_spacing, float mec_wheelbase, float mec_track_width);

    /**
     * @brief Inverse Kinematics Solver
     * @param vx    Linear velocity X (m/s)
     * @param vy    Linear velocity Y (m/s) - Ignored in CLIMBING mode
     * @param wz    Angular velocity Z (rad/s)
     * @param mode  CRUISING or CLIMBING
     * @return hybrid_speeds_t
     */
    [[nodiscard]] hybrid_speeds_t solve(float vx, float vy, float wz, drive_mode_t mode) const;

private:
    float _k_track; // track_spacing / 2
    float _k_mec;   // (wheelbase + track_width) / 2
};

} // namespace pyro

#endif
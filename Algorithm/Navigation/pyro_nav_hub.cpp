#include "pyro_nav_hub.h"

#include "pyro_dwt_drv.h"
#include <cmath>

namespace pyro
{

namespace
{
constexpr float NAV_EPS = 1.0e-6f;

float square(float value)
{
    return value * value;
}

bool nearly_equal(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) < NAV_EPS;
}

float clamp_dt(float dt, float min_dt, float max_dt)
{
    if (dt < min_dt)
    {
        return min_dt;
    }
    if (dt > max_dt)
    {
        return max_dt;
    }
    return dt;
}
} // namespace

nav_hub_t::nav_hub_t(const nav_point_t &initial_map_pos,
                     const nav_hub_deps_t &deps,
                     const nav_hub_config_t &config)
    : _deps(deps), _config(config), _current_pos(initial_map_pos)
{
    _clear_pid();
}

void nav_hub_t::reset(const nav_point_t &map_pos)
{
    _current_pos          = map_pos;
    _last_output.current  = _current_pos;
    _feedback_initialized = false;
    _dwt_cnt              = 0;
    _clear_pid();
}

void nav_hub_t::update_feedback(float vx, float vy, float yaw, float roll,
                                float pitch)
{
    _attitude = nav_attitude_t(yaw, roll, pitch);

    float dt = dwt_drv_t::get_delta_t(&_dwt_cnt);
    if (!_feedback_initialized)
    {
        _feedback_initialized = true;
        return;
    }

    dt = clamp_dt(dt, _config.min_dt, _config.max_dt);

    const nav_point_t ground_vel = body_to_ground_velocity(vx, vy, _attitude);
    _current_pos.x += ground_vel.x * dt;
    _current_pos.y += ground_vel.y * dt;
    _current_pos.z += ground_vel.z * dt;
}

bool nav_hub_t::set_target(float target_x, float target_y)
{
    const bool target_changed = !_target_valid ||
                                !nearly_equal(_target.x, target_x) ||
                                !nearly_equal(_target.y, target_y);

    _target       = nav_point_t(target_x, target_y, _current_pos.z);
    _target_valid = true;

    if (target_changed)
    {
        _clear_pid();
    }

    const float dist = _distance_to_target_xy(_target);
    if (dist < _config.arrive_radius)
    {
        _last_output.vx           = 0.0f;
        _last_output.vy           = 0.0f;
        _last_output.arrived      = true;
        _last_output.target_valid = true;
        _last_output.distance_xy  = dist;
        _last_output.current      = _current_pos;
        _last_output.target       = _target;
        _clear_pid();
        return true;
    }

    _last_output.arrived      = false;
    _last_output.target_valid = true;
    _last_output.distance_xy  = dist;
    _last_output.current      = _current_pos;
    _last_output.target       = _target;
    return false;
}

nav_output_t nav_hub_t::update()
{
    _last_output.current      = _current_pos;
    _last_output.target       = _target;
    _last_output.target_valid = _target_valid;

    if (!_target_valid)
    {
        _last_output.vx          = 0.0f;
        _last_output.vy          = 0.0f;
        _last_output.arrived     = true;
        _last_output.distance_xy = 0.0f;
        return _last_output;
    }

    const float dist = _distance_to_target_xy(_target);
    _last_output.distance_xy = dist;

    if (_deps.x_pid == nullptr || _deps.y_pid == nullptr)
    {
        _last_output.vx      = 0.0f;
        _last_output.vy      = 0.0f;
        _last_output.arrived = false;
        return _last_output;
    }

    if (dist < _config.arrive_radius)
    {
        _last_output.vx      = 0.0f;
        _last_output.vy      = 0.0f;
        _last_output.arrived = true;
        _clear_pid();
        return _last_output;
    }

    float ground_vx = _deps.x_pid->calculate(_target.x, _current_pos.x);
    float ground_vy = _deps.y_pid->calculate(_target.y, _current_pos.y);
    _limit_ground_speed(&ground_vx, &ground_vy);

    float body_vx = 0.0f;
    float body_vy = 0.0f;
    if (!ground_to_body_velocity(ground_vx, ground_vy, _attitude, &body_vx,
                                 &body_vy, _config.attitude_det_limit))
    {
        body_vx = 0.0f;
        body_vy = 0.0f;
    }

    _last_output.vx      = body_vx;
    _last_output.vy      = body_vy;
    _last_output.arrived = false;
    return _last_output;
}

void nav_hub_t::clear_target()
{
    _target_valid              = false;
    _last_output.vx            = 0.0f;
    _last_output.vy            = 0.0f;
    _last_output.arrived       = true;
    _last_output.target_valid  = false;
    _last_output.distance_xy   = 0.0f;
    _clear_pid();
}

const nav_point_t &nav_hub_t::get_current_pos() const
{
    return _current_pos;
}

const nav_point_t &nav_hub_t::get_target() const
{
    return _target;
}

const nav_attitude_t &nav_hub_t::get_attitude() const
{
    return _attitude;
}

bool nav_hub_t::has_target() const
{
    return _target_valid;
}

float nav_hub_t::get_distance_to_target_xy() const
{
    if (!_target_valid)
    {
        return 0.0f;
    }
    return _distance_to_target_xy(_target);
}

nav_point_t nav_hub_t::body_to_ground_velocity(float body_vx, float body_vy,
                                               const nav_attitude_t &attitude)
{
    const float cy = std::cos(attitude.yaw);
    const float sy = std::sin(attitude.yaw);
    const float cp = std::cos(attitude.pitch);
    const float sp = std::sin(attitude.pitch);
    const float cr = std::cos(attitude.roll);
    const float sr = std::sin(attitude.roll);

    nav_point_t ground;

    // R = Rz(yaw) * Ry(pitch) * Rx(roll).
    // Only the first two body basis vectors are needed for planar body vx/vy.
    ground.x = cy * cp * body_vx + (cy * sp * sr - sy * cr) * body_vy;
    ground.y = sy * cp * body_vx + (sy * sp * sr + cy * cr) * body_vy;
    ground.z = -sp * body_vx + cp * sr * body_vy;
    return ground;
}

bool nav_hub_t::ground_to_body_velocity(float ground_vx, float ground_vy,
                                        const nav_attitude_t &attitude,
                                        float *body_vx, float *body_vy,
                                        float det_limit)
{
    if (body_vx == nullptr || body_vy == nullptr)
    {
        return false;
    }

    const float cy = std::cos(attitude.yaw);
    const float sy = std::sin(attitude.yaw);
    const float cp = std::cos(attitude.pitch);
    const float sp = std::sin(attitude.pitch);
    const float cr = std::cos(attitude.roll);
    const float sr = std::sin(attitude.roll);

    const float a = cy * cp;
    const float b = cy * sp * sr - sy * cr;
    const float c = sy * cp;
    const float d = sy * sp * sr + cy * cr;
    const float det = a * d - b * c;

    if (std::fabs(det) < det_limit)
    {
        *body_vx = 0.0f;
        *body_vy = 0.0f;
        return false;
    }

    *body_vx = (d * ground_vx - b * ground_vy) / det;
    *body_vy = (-c * ground_vx + a * ground_vy) / det;
    return true;
}

float nav_hub_t::_distance_to_target_xy(const nav_point_t &target) const
{
    return std::sqrt(square(target.x - _current_pos.x) +
                     square(target.y - _current_pos.y));
}

void nav_hub_t::_clear_pid() const
{
    if (_deps.x_pid != nullptr)
    {
        _deps.x_pid->clear();
    }
    if (_deps.y_pid != nullptr)
    {
        _deps.y_pid->clear();
    }
}

void nav_hub_t::_limit_ground_speed(float *vx, float *vy) const
{
    if (vx == nullptr || vy == nullptr)
    {
        return;
    }

    const float norm = std::sqrt(square(*vx) + square(*vy));
    if (norm < NAV_EPS || norm <= _config.max_ground_speed)
    {
        return;
    }

    const float scale = _config.max_ground_speed / norm;
    *vx *= scale;
    *vy *= scale;
}

} // namespace pyro

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace pyro
{

constexpr uint8_t HEADER_SOF    = 0xA5;
constexpr size_t FRAME_MAX_SIZE = 256;
constexpr size_t HEADER_SIZE    = 5;
constexpr size_t CMD_SIZE       = 2;
constexpr size_t CRC16_SIZE     = 2;
constexpr size_t HEADER_CRC_LEN = HEADER_SIZE + CRC16_SIZE;
constexpr size_t HEADER_CRC_CMDID_LEN =
    HEADER_SIZE + CRC16_SIZE + sizeof(uint16_t);
constexpr size_t HEADER_CMDID_LEN = HEADER_SIZE + sizeof(uint16_t);

#pragma pack(push, 1)

enum class cmd_id : uint16_t
{
    GAME_STATE           = 0x0001,
    GAME_RESULT          = 0x0002,
    GAME_ROBOT_HP        = 0x0003,
    FIELD_EVENTS         = 0x0101,
    REFEREE_WARNING      = 0x0104,
    DART_INFO            = 0x0105,

    ROBOT_STATE          = 0x0201,
    POWER_HEAT_DATA      = 0x0202,
    ROBOT_POS            = 0x0203,
    BUFF_INFO            = 0x0204,
    ROBOT_HURT           = 0x0206,
    SHOOT_DATA           = 0x0207,
    PROJECTILE_ALLOWANCE = 0x0208,
    ROBOT_RFID           = 0x0209,
    DART_CLIENT_CMD      = 0x020A,
    GROUND_ROBOT_POS     = 0x020B,
    RADAR_MARK           = 0x020C,
    SENTRY_INFO          = 0x020D,
    RADAR_INFO           = 0x020E,

    STUDENT_INTERACTIVE  = 0x0301,
    CUSTOM_CONTROLLER    = 0x0302,
    TINY_MAP_INTERACT    = 0x0303,
    MAP_RECEIVE_RADAR    = 0x0305,
    CUSTOM_CLIENT_DATA   = 0x0306,
    MAP_RECEIVE_PATH     = 0x0307,
    MAP_RECEIVE_ROBOT    = 0x0308,

    BUFF_MUSK            = BUFF_INFO,
    BULLET_REMAINING     = PROJECTILE_ALLOWANCE,
};

enum class interaction_sub_cmd : uint16_t
{
    UI_CMD_DELETE    = 0x0100,
    UI_CMD_DRAW_1    = 0x0101,
    UI_CMD_DRAW_2    = 0x0102,
    UI_CMD_DRAW_5    = 0x0103,
    UI_CMD_DRAW_7    = 0x0104,
    UI_CMD_DRAW_CHAR = 0x0110,

    SENTRY_CMD       = 0x0120,
    RADAR_CMD        = 0x0121,

    ROBOT_COMM_START = 0x0200,
    ROBOT_COMM_END   = 0x02FF,
};

struct frame_header_t
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};

// 0x0001
struct game_status_t
{
    uint8_t game_type     : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

// 0x0002
struct game_result_t
{
    uint8_t winner;
};

// 0x0003
struct game_robot_hp_t
{
    uint16_t robot_1_hp;
    uint16_t robot_2_hp;
    uint16_t robot_3_hp;
    uint16_t robot_4_hp;
    uint16_t reserved;
    uint16_t robot_7_hp;
    uint16_t outpost_hp;
    uint16_t base_hp;
};

// 0x0101
struct event_data_t
{
    uint32_t supply_zone_occupied      : 1;
    uint32_t reserved_0                : 1;
    uint32_t rmul_supply_zone_occupied : 1;
    uint32_t small_energy_status       : 2;
    uint32_t big_energy_status         : 2;
    uint32_t circular_high_ground      : 2;
    uint32_t trapezoidal_high_ground   : 2;
    uint32_t dart_last_hit_time        : 9;
    uint32_t dart_last_hit_target      : 3;
    uint32_t center_buff_zone          : 2;
    uint32_t fortress_buff_zone        : 2;
    uint32_t outpost_buff_zone         : 2;
    uint32_t base_buff_zone            : 1;
    uint32_t reserved_1                : 2;
};

// 0x0104
struct referee_warning_t
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
};

// 0x0105
struct dart_info_t
{
    uint8_t dart_remaining_time;
    uint16_t dart_last_hit_target  : 3;
    uint16_t dart_target_hit_count : 3;
    uint16_t dart_aim_target       : 3;
    uint16_t reserved              : 7;
};

// 0x0201
struct robot_status_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t reserved                        : 5;
};

// 0x0202
struct power_heat_data_t
{
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};

// 0x0203
struct robot_pos_t
{
    float x;
    float y;
    float angle;
    uint32_t reserved;
};

// 0x0204
struct buff_info_t
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy_ge_125 : 1;
    uint8_t remaining_energy_ge_100 : 1;
    uint8_t remaining_energy_ge_50  : 1;
    uint8_t remaining_energy_ge_30  : 1;
    uint8_t remaining_energy_ge_15  : 1;
    uint8_t remaining_energy_ge_5   : 1;
    uint8_t remaining_energy_ge_1   : 1;
    uint8_t reserved                : 1;
};

// 0x0206
struct hurt_data_t
{
    uint8_t armor_id            : 4;
    uint8_t hp_deduction_reason : 4;
};

// 0x0207
struct shoot_data_t
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
};

// 0x0208
struct projectile_allowance_t
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
};

// 0x0209
struct rfid_status_t
{
    uint32_t ally_base_buff               : 1;
    uint32_t ally_circular_high           : 1;
    uint32_t enemy_circular_high          : 1;
    uint32_t ally_trapezoidal_high        : 1;
    uint32_t enemy_trapezoidal_high       : 1;
    uint32_t ally_fly_ramp_front          : 1;
    uint32_t ally_fly_ramp_back           : 1;
    uint32_t enemy_fly_ramp_front         : 1;
    uint32_t enemy_fly_ramp_back          : 1;
    uint32_t ally_circular_high_down      : 1;
    uint32_t ally_circular_high_up        : 1;
    uint32_t enemy_circular_high_down     : 1;
    uint32_t enemy_circular_high_up       : 1;
    uint32_t ally_highway_down            : 1;
    uint32_t ally_highway_up              : 1;
    uint32_t enemy_highway_down           : 1;
    uint32_t enemy_highway_up             : 1;
    uint32_t ally_fortress_buff           : 1;
    uint32_t ally_outpost_buff            : 1;
    uint32_t ally_supply_no_overlap       : 1;
    uint32_t ally_supply_overlap          : 1;
    uint32_t ally_assembly_buff           : 1;
    uint32_t enemy_assembly_buff          : 1;
    uint32_t center_buff                  : 1;
    uint32_t enemy_fortress_buff          : 1;
    uint32_t enemy_outpost_buff           : 1;
    uint32_t ally_tunnel_highway_down     : 1;
    uint32_t ally_tunnel_highway_mid      : 1;
    uint32_t ally_tunnel_highway_up       : 1;
    uint32_t ally_tunnel_trapezoidal_low  : 1;
    uint32_t ally_tunnel_trapezoidal_mid  : 1;
    uint32_t ally_tunnel_trapezoidal_high : 1;

    uint8_t enemy_tunnel_highway_down     : 1;
    uint8_t enemy_tunnel_highway_mid      : 1;
    uint8_t enemy_tunnel_highway_up       : 1;
    uint8_t enemy_tunnel_trapezoidal_low  : 1;
    uint8_t enemy_tunnel_trapezoidal_mid  : 1;
    uint8_t enemy_tunnel_trapezoidal_high : 1;
    uint8_t reserved                      : 2;
};

// 0x020A
struct dart_client_cmd_t
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
};

// 0x020B
struct ground_robot_position_t
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
};

// 0x020C
struct radar_mark_data_t
{
    uint16_t enemy_hero_marked       : 1;
    uint16_t enemy_engineer_marked   : 1;
    uint16_t enemy_infantry_3_marked : 1;
    uint16_t enemy_infantry_4_marked : 1;
    uint16_t enemy_aerial_marked     : 1;
    uint16_t enemy_sentry_marked     : 1;
    uint16_t ally_hero_marked        : 1;
    uint16_t ally_engineer_marked    : 1;
    uint16_t ally_infantry_3_marked  : 1;
    uint16_t ally_infantry_4_marked  : 1;
    uint16_t ally_aerial_marked      : 1;
    uint16_t ally_sentry_marked      : 1;
    uint16_t reserved                : 4;
};

// 0x020D
struct sentry_info_t
{
    uint32_t redeemed_projectile_allowance         : 11;
    uint32_t remote_projectile_exchange_count      : 4;
    uint32_t remote_hp_exchange_count              : 4;
    uint32_t free_respawn_available                : 1;
    uint32_t immediate_respawn_exchange_available  : 1;
    uint32_t immediate_respawn_cost                : 10;
    uint32_t reserved_1                            : 1;

    uint16_t out_of_combat                         : 1;
    uint16_t remaining_projectile_exchange_17mm    : 11;
    uint16_t sentry_posture                        : 2;
    uint16_t energy_mechanism_activation_available : 1;
    uint16_t reserved_2                            : 1;
};

// 0x020E
struct radar_info_t
{
    uint8_t double_vulnerability_chance : 2;
    uint8_t double_vulnerability_active : 1;
    uint8_t encryption_level            : 2;
    uint8_t key_can_be_changed          : 1;
    uint8_t reserved                    : 2;
};

// 0x0303
struct map_command_t
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
};

// 0x0305
struct map_robot_data_t
{
    uint16_t opponent_hero_position_x;
    uint16_t opponent_hero_position_y;
    uint16_t opponent_engineer_position_x;
    uint16_t opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x;
    uint16_t opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;
    uint16_t ally_hero_position_x;
    uint16_t ally_hero_position_y;
    uint16_t ally_engineer_position_x;
    uint16_t ally_engineer_position_y;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t ally_aerial_position_x;
    uint16_t ally_aerial_position_y;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
};

// 0x0307
struct map_data_t
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
};

// 0x0301 header
struct interaction_header_t
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

// 0x0301 payload wrapper
struct robot_interaction_data_t
{
    interaction_header_t header;
    uint8_t user_data[112];
};

// 0x0120
struct sentry_cmd_t
{
    uint32_t confirm_respawn                     : 1;
    uint32_t confirm_immediate_respawn_exchange  : 1;
    uint32_t projectile_allowance_to_exchange    : 11;
    uint32_t remote_projectile_exchange_count    : 4;
    uint32_t remote_hp_exchange_count            : 4;
    uint32_t sentry_posture                      : 2;
    uint32_t confirm_energy_mechanism_activation : 1;
    uint32_t reserved                            : 8;
};

// 0x0121
struct radar_cmd_t
{
    uint8_t double_vulnerability_trigger_count;
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
};

// 0x0308
struct custom_info_t
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
};

struct referee_data_t
{
    game_status_t game_status;
    game_result_t game_result;
    game_robot_hp_t game_robot_hp;
    event_data_t field_event;
    referee_warning_t referee_warning;
    dart_info_t dart_info;

    robot_status_t robot_status;
    power_heat_data_t power_heat;
    robot_pos_t robot_pos;
    buff_info_t buff;
    hurt_data_t hurt;
    shoot_data_t shoot;
    uint16_t shoot_launching_count;
    projectile_allowance_t allowance;
    rfid_status_t rfid;
    dart_client_cmd_t dart_client_cmd;
    ground_robot_position_t ground_robot_pos;
    radar_mark_data_t radar_mark;
    sentry_info_t sentry_info;
    radar_info_t radar_info;

    map_command_t map_command;
    robot_interaction_data_t robot_interaction;
};

#pragma pack(pop)

} // namespace pyro

#endif // ROBOMASTER_PROTOCOL_H

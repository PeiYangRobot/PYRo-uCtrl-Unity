#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"
#include "stdbool.h"

#define GAME_MODE 1

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef struct __packed  //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} game_status_t;

typedef struct __packed 	//0002
{
	uint8_t winner;
}game_result_t;
typedef struct __packed 	//0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t reserved_1;
    uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved_2;
    uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
typedef struct __packed  //0101
{
    uint32_t event_type;
} event_data_t;

typedef struct __packed  //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef struct __packed //0x0104
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
}referee_warning_t;

typedef struct __packed //0x0201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP; 
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_shooter_output : 1;
}robot_status_t;

typedef struct __packed 	//0x0202
{
	uint16_t reserved_1;
	uint16_t reserved_2;
	float reserved_3;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

typedef struct __packed  //0x0203
{
    float x;
    float y;
    float yaw;
} robot_pos_t;

typedef struct __packed  //0x0204
{
    uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;
} buff_t;

typedef struct __packed 	//0x0205
{
	uint8_t airforce_status;
	uint8_t time_remain;
}air_support_data_t;

typedef struct __packed  //0x0206
{
    uint8_t armor_type : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

typedef struct __packed 	//0x0207
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
}shoot_data_t;

typedef struct __packed 	//0x0208
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
}projectile_allowance_t;

typedef struct __packed 	//0x0209
{
	uint32_t rfid_status;
}rfid_status_t;

typedef struct __packed 	//0x020B
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float reseved_1;
	float reserved_2;
}ground_robot_position_t;

typedef struct __packed 		//0x020D
{
	uint32_t exchanged_bullet_successful:11;
	uint32_t remote_exchanged_bullet_times:4;
	uint32_t remote_exchanged_HP_times:4;
	uint32_t revive_free_dekilu:1;
	uint32_t revive_instant_dekilu:1;
	uint32_t revive_instant_cost_golden:10;
	uint32_t reseved_1:1;
	uint16_t out_of_battlefield:1;
	uint16_t remain_team_total_17mm_exchangealbe:11;
	uint16_t reserved_2:4;
}sentry_info_t;


typedef struct __packed 	//0x0307
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
}map_data_t;

typedef struct __packed 	//0x0308
{ 
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

typedef struct __packed
{
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t sentry_cmd[4];      //uint8_t user_data[x];对哨兵自主决策来说x=4
}robot_interaction_data_t;

typedef struct __packed
{
    uint8_t  sof;
    uint16_t dataLenth;
    uint8_t  seq;
    uint8_t  crc8;
} tFrameHeader;

typedef struct __packed
{
	tFrameHeader FrameHead;										   // 帧头
	uint16_t CmdId;												   // 命令码
	robot_interaction_data_t sentry_decision;
	uint16_t CRC16;
} sentry_desicion_making_t;


typedef struct	
{
	game_status_t game_status;//1
	game_result_t	 game_result;//2
	ext_game_robot_HP_t game_robot_HP;//3
	event_data_t field_event;//101
	ext_supply_projectile_action_t projectile;//102
	referee_warning_t referee_warning;//104
	robot_status_t robot_status;//201
	power_heat_data_t power_heat;//202
	robot_pos_t robot_pos;//203
	buff_t buff;//204
	hurt_data_t hurt;//206
	shoot_data_t shoot;//207
	projectile_allowance_t allowance;//208
	rfid_status_t rfid;//209
	ground_robot_position_t ground_robot_position;//20B
	sentry_info_t sentry_info;//20D
	robot_interaction_data_t robot_interaction;//301
	map_data_t map;//307
	custom_info_t custom_info;//308
}referee_data_t;

extern referee_data_t referee_data;



extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer( fp32 *buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

extern bool check_game_start(void);
extern void revival_pending_flag_bind(uint8_t *flag_addr);
extern void revival_pending_flag(void);
extern void Referee_Check_Hurt(void);
extern uint8_t Referee_Check_hurt_gyro_flag(void);
extern uint8_t Referee_Check_Revival_Status();

extern referee_data_t referee_data;
#endif

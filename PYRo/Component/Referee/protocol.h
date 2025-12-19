#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,//比赛状态信息
    GAME_RESULT_CMD_ID                = 0x0002,//比赛结果数据
    GAME_ROBOT_HP_CMD_ID              = 0x0003,//机器人血量
    FIELD_EVENTS_CMD_ID               = 0x0101,//场地事件数据
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,//补给站数据
    //SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,//裁判警告数据

    DART_EMMIT_CMD_ID	              = 0x0105,//飞镖发射相关数据

    ROBOT_STATE_CMD_ID                = 0x0201,//机器人性能数据
    POWER_HEAT_DATA_CMD_ID            = 0x0202,//底盘功率和枪口热量
    ROBOT_POS_CMD_ID                  = 0x0203,//机器人位置信息
    BUFF_MUSK_CMD_ID                  = 0x0204,//机器人增益
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,//空中支援时间数据
    ROBOT_HURT_CMD_ID                 = 0x0206,//伤害状态数据
    SHOOT_DATA_CMD_ID                 = 0x0207,//实时射击数据
    BULLET_REMAINING_CMD_ID           = 0x0208,//允许发弹量

    ROBOT_RFID_CMD_ID	              = 0x0209,//机器人RFID数据

    SENTRY_MATE_POS_CMD_ID            = 0x020B,//地面机器人数据
    SENTRY_SELF_DECISION_CMD_ID	      = 0x020D,//哨兵自主决策信息同步


    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,//机器人交互数据

    TINY_MAP_INTERACT_CMD_ID          = 0x0303,//选手端小地图交互数据
    TINY_MAP_RECEIVE_SENTRY_CMD_ID    = 0x0307,//小地图接收哨兵数据
    TINY_MAP_RECEIVE_ROBOT_CMD_ID     = 0x0308,//选手端小地图接受机器人数据
}referee_cmd_id_t;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H

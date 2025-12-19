#include "referee.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "stdio.h"
#include "string.h"

extern void shoot_send(float shoot_speed);


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

referee_data_t referee_data;

void init_referee_struct_data(void)
{
    memset(&referee_data,0,sizeof(referee_data));
}

uint8_t first_commit_flag=0;
uint8_t armor_bullet_hurt_flag =0;

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&referee_data.game_status, frame + index, sizeof(referee_data.game_status));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&referee_data.game_result, frame + index, sizeof(referee_data.game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&referee_data.game_robot_HP, frame + index, sizeof(referee_data.game_robot_HP));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&referee_data.field_event, frame + index, sizeof(referee_data.field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&referee_data.projectile, frame + index, sizeof(referee_data.projectile));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_data.referee_warning, frame + index, sizeof(referee_data.referee_warning));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&referee_data.robot_status, frame + index, sizeof(referee_data.robot_status));
						first_commit_flag=1;
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&referee_data.power_heat, frame + index, sizeof(referee_data.power_heat));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&referee_data.robot_pos, frame + index, sizeof(referee_data.robot_pos));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&referee_data.buff, frame + index, sizeof(referee_data.buff));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&referee_data.hurt, frame + index, sizeof(referee_data.hurt));
					if(referee_data.hurt.HP_deduction_reason==0)
					{
						armor_bullet_hurt_flag=1;
					}
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&referee_data.shoot, frame + index, sizeof(referee_data.shoot));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&referee_data.allowance, frame + index, sizeof(referee_data.allowance));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&referee_data.robot_interaction, frame + index, sizeof(referee_data.robot_interaction));
        }
        break;
				case ROBOT_RFID_CMD_ID:
        {
            memcpy(&referee_data.rfid, frame + index, sizeof(referee_data.rfid));
        }
        break;
				case SENTRY_MATE_POS_CMD_ID:
        {
            memcpy(&referee_data.ground_robot_position, frame + index, sizeof(referee_data.ground_robot_position));
        }
        break;
				case SENTRY_SELF_DECISION_CMD_ID:
        {
            memcpy(&referee_data.sentry_info, frame + index, sizeof(referee_data.sentry_info));
        }
        break;
				case TINY_MAP_RECEIVE_SENTRY_CMD_ID:
        {
            memcpy(&referee_data.map, frame + index, sizeof(referee_data.map));
        }
        break;
        default:
        {
            break;
        }
    }
}





#ifndef PYRO_STRUCT_H
#define PYRO_STRUCT_H


#include <cstdint>

#pragma pack(push, 1)

struct FrameHeader
{
    uint8_t sof;
};

struct FrameTailer
{
    uint16_t crc16;
    uint8_t end;
};

struct OutputData
{
    float curr_yaw;
    float curr_pitch;
    uint8_t state;
    uint8_t autoaim;
    uint8_t enemy_color;
    float curr_speed;
    uint16_t shoot_delay;
};

struct InputData
{
    uint8_t fire;
    float shoot_yaw;
    float shoot_pitch;
    float avg_speed;
    uint8_t food;
};

struct StateBytes
{
    FrameHeader frame_header;
    InputData input_data;
    FrameTailer frame_tailer;
};

struct OperateBytes
{
    FrameHeader frame_header;
    OutputData output_data;
    FrameTailer frame_tailer;
};

#pragma pack(pop)

#endif // PYRO_STRUCT_H

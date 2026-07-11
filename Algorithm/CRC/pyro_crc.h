#ifndef __PYRO_CORE_PROTOCOL_H__
#define __PYRO_CORE_PROTOCOL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 校验 CRC16
 * @return 1 (true) if valid, 0 (false) if invalid
 */
extern uint8_t verify_crc16_check_sum(uint8_t const* p_msg, uint16_t len);

/**
 * @brief 添加 CRC16 到缓冲区末尾
 * @note  缓冲区长度必须包含最后2个字节的空间
 */
extern void append_crc16_check_sum(uint8_t* p_msg, uint16_t len);

/**
 * @brief 校验 CRC8
 * @return 1 (true) if valid, 0 (false) if invalid
 */
extern uint8_t verify_crc8_check_sum(uint8_t const* p_msg, uint16_t len);

/**
 * @brief 添加 CRC8 到缓冲区末尾
 * @note  缓冲区长度必须包含最后1个字节的空间
 */
extern void append_crc8_check_sum(uint8_t* p_msg, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
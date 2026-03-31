/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-01-29 20:00:33
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-01-30 13:09:02
 * @FilePath: \PYRo-uCtrl-Unity-dev\PYRo\Device\BMI088\BMI088_middleware.h
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BMI088_MIDDLEWARE_H
#define BMI088_MIDDLEWARE_H

#include "main.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

#define CS1_ACCEL_Pin GPIO_PIN_0
#define CS1_ACCEL_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_3
#define CS1_GYRO_GPIO_Port GPIOC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

extern SPI_HandleTypeDef *BMI088_SPI;

#elif defined(BMI088_USE_IIC)

#endif

#endif
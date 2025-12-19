#ifndef __IMU_CONTROL_H__
#define __IMU_CONTROL_H__

#include"BMI088driver.h"
#include"AHRS.h"
#include <stdint.h>

#define CALI_MODE 0

//任务开始延时
#define IMU_TASK_INIT_TIME 3
//最大PWM占空比
#define MPU6500_TEMP_PWM_MAX 5000
//开始校准时间
#define START_CALI_TIME 5000
//最大校准时间
#define MAX_CALI_TIME 10000
//陀螺仪接受补偿
#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
//加速度计接收补偿
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2
//Yaw轴角度相对于头的偏移量
#define IMU_YAW_ADDRESS_OFFSET    0
//Pitch轴角度相对于头的偏移量
#define IMU_PITCH_ADDRESS_OFFSET  1
//Roll轴角度相对于头的偏移量
#define IMU_ROLL_ADDRESS_OFFSET   2
typedef enum
{
	CALI_ON=1,
	CALI_FINISH
}
cali_mode_t;


typedef struct imu
{
	//member variable
	float gyro[3];
	float accel[3];

	float quaternion[4];
	float roll;
	float pitch;
	float yaw;
	
	//method
	void (*Update)(struct imu* _this);
	float (*Get_Roll)(struct imu* _this);
	float (*Get_Pitch)(struct imu* _this);
	float (*Get_Yaw)(struct imu* _this);
}IMU_obj;

void IMU_Base_Update(IMU_obj* _this);
float IMU_Base_Get_Roll(IMU_obj* _this);
float IMU_Base_Get_Pitch(IMU_obj* _this);
float IMU_Base_Get_Yaw(IMU_obj* _this);

#ifdef __cplusplus  
extern "C" {        
#endif

extern IMU_obj* IMU_Base_Factory_Function(void);
void imu_slove(float gyro[3], float accel[3], bmi088_real_data_t *bmi088);
void gyro_cali(float32_t gyro[3]);
#ifdef __cplusplus
}  
#endif


extern bmi088_real_data_t bmi088_real_data;
extern float imu_gyro[3];
extern float imu_accel[3];
extern float imu_quat[4];
extern float32_t accel_fliter_1[3];
extern float32_t accel_fliter_2[3];
extern float32_t accel_fliter_3[3];
extern const float32_t fliter_num[3];
extern float imu_rad[3]; 
extern float imu_angle[3];

#endif
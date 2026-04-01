#include "pyro_core_config.h"
#if IMU_DEMO_EN
#include"BMI088driver.h"
#include"PID.h"
#include"AHRS.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include <string.h>
#include "MATH_LIB.h"
#include"IMU_Base.h"

#ifdef __cplusplus
extern "C"
{

extern IMU_obj Imu;

uint8_t temp_ctrl=0;
    
void IMU_task(void * argument)
{
	//等待陀螺仪启动
	IMU_Base_Factory_Function();
	osDelay(500);
	osDelay(IMU_TASK_INIT_TIME);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
    while(BMI088_init())
    {
        ;
    }
	//SPI读取数据
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //计算零漂
    imu_slove(imu_gyro, imu_accel, &bmi088_real_data);
	//控温PID初始化
    //PID_Init(&imu_temp_pid,TEMP_PID_K[0],TEMP_PID_K[1],TEMP_PID_K[2],TEMP_PID_IMAX,TEMP_PID_MAXOUT,	0,0,0,	0,0,0,		0,		0,0);
	//四元数初始化
	AHRS_init(imu_quat);
	//低通滤波器启动
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_accel[2];
    while (1)
    {	
		#if CALI_MODE
		gyro_cali(imu_gyro);
		#endif
		// __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,temp_ctrl);
				
		BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
		//计算角速度与加速度
        imu_slove(imu_gyro, imu_accel, &bmi088_real_data);
        //加速度计低通滤波
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];
        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_accel[0] * fliter_num[2];
        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];
        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_accel[1] * fliter_num[2];
        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];
        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + imu_accel[2] * fliter_num[2];
		//四元数更新
		AHRS_update(imu_quat, imu_gyro, accel_fliter_3);
		AHRS_get(imu_quat, imu_rad + IMU_YAW_ADDRESS_OFFSET, imu_rad + IMU_PITCH_ADDRESS_OFFSET, imu_rad + IMU_ROLL_ADDRESS_OFFSET);
		for(int i=0;i<3;i++)
		{
			imu_angle[i] = imu_rad[i]/ PI * 180.0f;
		}
		Imu.Update(&Imu);
		vTaskDelay(1);
				
				
		#if CALI_MODE
				gyro_cali(imu_gyro);
		#endif
	}
}

}

#endif
#endif

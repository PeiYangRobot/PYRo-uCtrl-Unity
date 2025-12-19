#include"PID.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include <string.h>
#include "MATH_LIB.h"
#include"IMU_Base.h"

//#include "Debug\VOFA/pyro_vofa.h"
void spi2_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void spi2_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static void imu_cmd_spi_dma(void);
void IMU_Stream1_IRQHandler(void);
void gyro_cali(float32_t gyro[3]);
void imu_slove(float gyro[3], float accel[3], bmi088_real_data_t *bmi088);
static void imu_temp_control(float32_t temp);
static void imu_temp_pwm(uint16_t pwm);

TaskHandle_t imu_task_local_handler;

///////////////////////板级支持包///////////////////////////////////
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin == INT1_ACCEL_Pin)//IMU的加速度计和温度数据准备发送挂起标志位
//    {
//			
//				if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
//        {
//            static BaseType_t xHigherPriorityTaskWoken;
//            vTaskNotifyGiveFromISR(imu_task_local_handler, &xHigherPriorityTaskWoken);//在中断向指定的任务发送一个通知
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//没懂
//        }
//    }
// 
//}
//校准标志位
uint8_t cali_flag = CALI_ON;
//校准时间
uint32_t cali_time = 0;
//校准值
float32_t cali_offset[3] = {0, 0, 0};
//理论校准值
float32_t manual_offset[3] = {-0.00520456877, 0.00210196408, -0.000137845083};

//陀螺仪校准
void gyro_cali(float32_t gyro[3])
{
	uint8_t i = 0;
	//在校准模式下
	if(cali_flag == CALI_ON)
	{
		//校准时间自增
		cali_time++;
		//当时间大于校准允许时间时
		if(cali_time >= START_CALI_TIME)
		{
			//校准值多次累加
			cali_offset[0] += gyro[0];
			cali_offset[1] += gyro[1];
			cali_offset[2] += gyro[2];
		}
		if(cali_time >= MAX_CALI_TIME)
		{
			for(i = 0; i < 3; i++)
			//校准值取平均值
			cali_offset[i] /= (float)(MAX_CALI_TIME - START_CALI_TIME);
			//校准完成
			cali_flag = CALI_FINISH;
		}
	}
}

//BMI088安装矩阵
#define BMI088_BOARD_INSTALL_SPIN_MATRIX	\
{0.0f, 1.0f, 0.0f},           				\
{-1.0f, 0.0f, 0.0f},                   		\
{0.0f, 0.0f, 1.0f}

//BMI088接收原始格式数据
bmi088_real_data_t bmi088_real_data;
//跟IMU安装朝向有关
float32_t gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
//陀螺仪补偿值
float32_t gyro_offset[3];
//陀螺仪校准补偿值
float32_t gyro_cali_offset[3];
//跟IMU朝向有关
float32_t accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
//加速度补偿值
float32_t accel_offset[3];
//加速度校准补偿值
float32_t accel_cali_offset[3];

//IMU数据处理
void imu_slove(float gyro[3], float accel[3], bmi088_real_data_t *bmi088)
{
	uint8_t i = 0;
    for (i = 0; i < 3; i++)
    {
			//将数据按旋转矩阵旋转
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];	
	}
	#if CALI_MODE
	if(cali_flag == CALI_FINISH) 
	{
		for(i = 0; i< 3; i++)
			gyro[i] = gyro[i] - cali_offset[i];
	}
	#else
	for(i = 0; i< 3; i++)
		//实际值减去校准值
		gyro[i] = gyro[i] - manual_offset[i];
	#endif
}

uint8_t first_temperate;
 float output_temp=0.0f;
 PID_t imu_temp_pid;

extern TIM_HandleTypeDef htim3;

#define TEMP_PID_KP (float)0.0f
#define TEMP_PID_KI (float)0.0f
#define TEMP_PID_KD (float)0.0f
#define TEMP_PID_IMAX (float)0.0f
#define TEMP_PID_MAXOUT (float)0.0f
float TEMP_PID_K[3]={TEMP_PID_KP,TEMP_PID_KI,TEMP_PID_KD};


//BMI088控温
static void imu_temp_control(float32_t temp)
{
    uint16_t temp_pwm;
    static uint8_t temp_constant_time = 0;
			//在第一次温度控制之后使用闭环温度控制
    if (first_temperate)
    {
        //PID_Predict(&imu_temp_pid, temp, 40.0f,&output_temp);
        if (output_temp < 0.0f)
        {
            output_temp = 0.0f;
        }
        temp_pwm = (uint16_t)output_temp;
        imu_temp_pwm(temp_pwm);
    }
		//第一次温控时进行最大功率开环控制
    else
    {
        //首次进行温度闭环，一直最大功率加热
        if (temp > 40.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        imu_temp_pwm(MPU6500_TEMP_PWM_MAX - 1);
    }
}

extern TIM_HandleTypeDef htim3;

//IMU加热
static void imu_temp_pwm(uint16_t pwm)
{
	//PWM波进行加热
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm);
}

//加速度计含有高频噪声，用低通滤波器进行滤波
 float32_t accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
 float32_t accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
 float32_t accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
//滤波器权重
const float32_t fliter_num[3] = {0.1251596093291,   0.5834061966847,   0.2914341939862};

//陀螺仪数据
float imu_gyro[3] = {0.0f, 0.0f, 0.0f};
//加速度计数据
float imu_accel[3] = {0.0f, 0.0f, 0.0f};
//四元数
float imu_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//欧拉角	弧度制
float imu_rad[3] = {0.0f, 0.0f, 0.0f};  
//欧拉角 角度制
float imu_angle[3]={0.0f,0.0f,0.0f};

IMU_obj Imu;

int total_cali_time=300;
int cali_count=-1;
float last_val,this_val;
float cali_array[500];
double cali_average;

float minor_diff=6.643577448067e-05;

void yaw_cali(void)
{
	if(cali_count==-1)
	{
		last_val=imu_angle[0];
		cali_count++;
		return ;
	}
	else
	{
		last_val=this_val;
	}
	this_val=imu_angle[0];
	
	if(cali_count<total_cali_time)
	{
		if(this_val-last_val<-180)
			cali_array[cali_count]=this_val-last_val+360;
		else if(this_val-last_val>180)
			cali_array[cali_count]=this_val-last_val-360;
		else
			cali_array[cali_count]=this_val-last_val;
		cali_count++;
	}
	else if(cali_count==total_cali_time)
	{
		for(int i=0;i<total_cali_time;i++)
		{
			cali_average+=cali_array[i];
		}
		cali_average/=total_cali_time;
	}
}

// uint8_t temp_ctrl=0;

// void IMU_task(void * argument)
// {
// 	//等待陀螺仪启动
// 	//vTaskDelay
// 	//gyro_cali(imu_gyro);
// 	//vTaskDelay(20);
// 	IMU_Base_Factory_Function();
// 	osDelay(500);
// 	osDelay(IMU_TASK_INIT_TIME);
// 	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
//     while(BMI088_init())
//     {
//         ;
//     }
// 	//SPI读取数据
//     BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
//     //计算零漂
//     imu_slove(imu_gyro, imu_accel, &bmi088_real_data);
// 	//控温PID初始化
//     //PID_Init(&imu_temp_pid,TEMP_PID_K[0],TEMP_PID_K[1],TEMP_PID_K[2],TEMP_PID_IMAX,TEMP_PID_MAXOUT,	0,0,0,	0,0,0,		0,		0,0);
// 	//四元数初始化
// 	AHRS_init(imu_quat);
// 	//低通滤波器启动
//     accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_accel[0];
//     accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_accel[1];
//     accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_accel[2];
//    //imu_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
//     while (1)
//     {	
// 		//while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
//         //{
//         //}
// 		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,temp_ctrl);
				
// 		BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
// 		//计算角速度与加速度
//         imu_slove(imu_gyro, imu_accel, &bmi088_real_data);
//         //加速度计低通滤波
//         accel_fliter_1[0] = accel_fliter_2[0];
//         accel_fliter_2[0] = accel_fliter_3[0];
//         accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_accel[0] * fliter_num[2];
//         accel_fliter_1[1] = accel_fliter_2[1];
//         accel_fliter_2[1] = accel_fliter_3[1];
//         accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_accel[1] * fliter_num[2];
//         accel_fliter_1[2] = accel_fliter_2[2];
//         accel_fliter_2[2] = accel_fliter_3[2];
//         accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + imu_accel[2] * fliter_num[2];
// 		//四元数更新
// 		AHRS_update(imu_quat, imu_gyro, accel_fliter_3);
// 		AHRS_get(imu_quat, imu_rad + IMU_YAW_ADDRESS_OFFSET, imu_rad + IMU_PITCH_ADDRESS_OFFSET, imu_rad + IMU_ROLL_ADDRESS_OFFSET);
// 		for(int i=0;i<3;i++)
// 		{
// 			imu_angle[i] = imu_rad[i]/ PI * 180.0f;
// 		}
// 		//imu_angle[0]-=minor_diff;
// //		Justfloat_Send
// //		(
// //		imu_gyro[0],imu_gyro[1],imu_gyro[2],
// //		imu_accel[0],imu_accel[1],imu_accel[2],
// //		accel_fliter_3[0],accel_fliter_3[1],accel_fliter_3[2],
// //		imu_angle[0],imu_angle[1],imu_angle[2]
// //		);
// 		//yaw_cali();
// 		vTaskDelay(1);
				
				
// 		#if CALI_MODE
// 				gyro_cali(imu_gyro);
// 		#endif
// 	}
// }

//IMUUpdate默认实现
void IMU_Base_Update(IMU_obj* this)
{
	memcpy(this->gyro,imu_gyro,sizeof(float)*3);
	memcpy(this->accel,imu_accel,sizeof(float)*3);
	memcpy(this->quaternion,imu_quat,sizeof(float)*4);
	this->roll=imu_rad[2];
	this->pitch=imu_rad[1];
	this->yaw=imu_rad[0];
}

//IMU获取Roll默认实现
float IMU_Base_Get_Roll(IMU_obj* this)
{
	return this->roll;
}

//IMU获取Pitch默认实现
float IMU_Base_Get_Pitch(IMU_obj* this)
{
	return this->pitch;
}

//IMU获取Yaw默认实现
float IMU_Base_Get_Yaw(IMU_obj* this)
{
	return this->yaw;
}

//IMU对象工厂方法
IMU_obj* IMU_Base_Factory_Function(void)
{
	Imu.Update=IMU_Base_Update;
	Imu.Get_Roll=IMU_Base_Get_Roll;
	Imu.Get_Pitch=IMU_Base_Get_Pitch;
	Imu.Get_Yaw=IMU_Base_Get_Yaw;
	
	return &Imu;
}
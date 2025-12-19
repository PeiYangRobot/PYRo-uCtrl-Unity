#include "IMU_Ext.h"

IMU_obj* IMU_Ext_Type1_Factory(void)
{
    IMU_obj* imu = (IMU_obj*)malloc(sizeof(IMU_obj));
    imu->Get_Pitch = IMU_Base_Get_Pitch;
    imu->Get_Roll = IMU_Base_Get_Roll;
    imu->Get_Yaw = IMU_Base_Get_Yaw;
    imu->Update = IMU_Base_Update;
    return imu;
}

void IMU_Ext_Type1_Update(IMU_obj*this)
{

}
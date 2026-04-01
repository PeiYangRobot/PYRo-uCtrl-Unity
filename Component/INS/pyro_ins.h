#ifndef PYRO_INS_H
#define PYRO_INS_H

#include "pyro_core_config.h"
#include "pyro_core_def.h"
#include "BMI088_driver.h"
#include "FreeRTOS.h"
#include "task.h"
namespace pyro {

class ins_drv_t {
private:
    IMU_Data_t imu_data;
    //quaternion=q0+i*q1+j*q2+k*q3
    float _q[4];
    float _gyro_b[3];
    float _gyro_n[3];
    float _acc_b[3];
    float _acc_n[3];
    float _angle_b[3];
    float _angle_n[3];

    float _dt;
    float _t;
    uint32_t _dwt_cnt;
    static TaskHandle_t _ins_task_handle;
    void __ins_task();
    static void __static_ins_task(void *argument);
    
public:
    static ins_drv_t* get_instance(void);
    status_t init();
    status_t get_angles_b(float *yaw, float *pitch, float *roll);
    status_t get_angles_n(float *yaw, float *pitch, float *roll);
    status_t get_rads_b(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_rads_n(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_gyro_b(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_gyro_n(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_accel_b(float* accel_x, float* accel_y, float* accel_z);
    status_t get_accel_n(float* accel_x, float* accel_y, float* accel_z);
    status_t get_quaternion(float* q0, float* q1, float* q2, float* q3);
};
}
#endif

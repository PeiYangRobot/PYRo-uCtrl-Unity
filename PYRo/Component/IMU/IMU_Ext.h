#ifndef __IMU_EXT_H__
#define __IMU_EXT_H__

#include "main.h"
#include "IMU_Base.h"
#include "cmsis_os.h"

#define malloc pvPortMalloc

IMU_obj* IMU_Ext_Type1_Factory(void);
void IMU_Ext_Type1_Update(IMU_obj* obj);

#endif
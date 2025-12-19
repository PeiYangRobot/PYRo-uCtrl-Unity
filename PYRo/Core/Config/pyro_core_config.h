#ifndef __PYRO_CORE_CONFIG_H__
#define __PYRO_CORE_CONFIG_H__

#define DEMO_MODE 1
#define DEBUG_MODE 1

#if DEMO_MODE

#define RC_DEMO_EN 0
#define MOTOR_DEMO_EN 0
#define CONTROLLER_DEMO_EN 0
#define CONTROL_DEMO_EN 0
#define IMU_DEMO_EN 0
#define REFEREE_DEMO_EN 0

#endif

#if DEBUG_MODE

#define VOFA_DEBUG_EN 0
#define JCOM_DEBUG_EN 0

#endif


#endif //PYRO_PYRO_CORE_CONFIG_H
#ifndef LMP_PROJECT_HARDWARE_MECANUM_WHEEL_CONTROL_H_
#define LMP_PROJECT_HARDWARE_MECANUM_WHEEL_CONTROL_H_

#include "mecanum/config.h"

#define DUTYCYCLE_RANGE 255U
#define FREQUENCY 1000U

#define FRONT_LEFT_IN1 GPIO_UNASSIGNED
#define FRONT_LEFT_IN2 GPIO_UNASSIGNED

#define FRONT_RIGHT_IN1 GPIO_UNASSIGNED
#define FRONT_RIGHT_IN2 GPIO_UNASSIGNED

#define REAR_LEFT_IN1 GPIO_UNASSIGNED
#define REAR_LEFT_IN2 GPIO_UNASSIGNED

#define REAR_RIGHT_IN1 GPIO_UNASSIGNED
#define REAR_RIGHT_IN2 GPIO_UNASSIGNED

typedef struct {
    unsigned int in1;
    unsigned int in2;
} MotorDriveGPIO;

#ifdef DEBUG
typedef struct {
    MotorDriveGPIO motordrive;
    bool initialized;
    const uint8_t index;
} MotorDriveInfo;

extern MotorDriveInfo WHEELS[ROBOT_MANAGED_WHEEL_COUNT];
#else

typedef struct {
    MotorDriveGPIO motordrive;
    bool initialized;
} MotorDriveInfo;

extern MotorDriveInfo WHEELS[ROBOT_MANAGED_WHEEL_COUNT];
#endif //DEBUG

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

int init_wheel(MotorDriveInfo* target);

int forward(const MotorDriveInfo* target, unsigned int duty);

int reverse(const MotorDriveInfo* target, unsigned int duty);

int idle(const MotorDriveInfo* target);

int brake(const MotorDriveInfo* target);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //LMP_PROJECT_HARDWARE_MECANUM_WHEEL_CONTROL_H_

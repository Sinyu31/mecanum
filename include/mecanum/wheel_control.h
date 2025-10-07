#ifndef LMP_PROJECT_HARDWARE_MECANUM_WHEEL_CONTROL_H_
#define LMP_PROJECT_HARDWARE_MECANUM_WHEEL_CONTROL_H_

#include "mecanum/daemon.h"

/**
 * @file wheel_control.h
 * @brief Provides control of mecanum wheels mounted on DC motors
 *
 * This module performs wheel initialization and wheel control (forward, reverse, idle, brake) for each mecanum wheel motor
*/

/* Constants */
#define DUTYCYCLE_RANGE 255U
#define FREQUENCY 1000U

/* GPIO Configuration */
#define FRONT_LEFT_IN1 GPIO_UNASSIGNED  //12
#define FRONT_LEFT_IN2 GPIO_UNASSIGNED  //16

#define FRONT_RIGHT_IN1 GPIO_UNASSIGNED //20
#define FRONT_RIGHT_IN2 GPIO_UNASSIGNED //21

#define REAR_LEFT_IN1 GPIO_UNASSIGNED   //5
#define REAR_LEFT_IN2 GPIO_UNASSIGNED   //6

#define REAR_RIGHT_IN1 GPIO_UNASSIGNED  //13
#define REAR_RIGHT_IN2 GPIO_UNASSIGNED  //19

/**
 * @struct MotorDriveGPIO
 * @brief Holds motor driver output pin numbers
*/
typedef struct {
    unsigned int in1; //in1
    unsigned int in2; //in2
} MotorDriveGPIO;

#ifdef DEBUG
/**
 * @struct MotorDriveInfo
 * @brief Motor driver information with debug index 
*/
typedef struct {
    MotorDriveGPIO motordrive; //Motor driver pins
    bool initialized;          //Initialization status
    const uint8_t index;       //Wheel index (use debug)
} MotorDriveInfo;

#else
/**
 * @struct MotorDriveInfo
 * @brief Motor driver information (release version)
*/
typedef struct {
    MotorDriveGPIO motordrive; //Motor driver pins
    bool initialized;          //Initialization status
} MotorDriveInfo;

#endif //DEBUG

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

extern MotorDriveInfo WHEELS[ROBOT_MANAGED_WHEEL_COUNT];

/**
 * @brief Initialize a wheel motor driver
 *
 * @param pi pigpid demon handle
 * @param target Target wheel motor driver (e.g WHEELS[0])
 * @return RC_OK if OK, otherwise RC_ALREADY_INITIALIZED or RC_INVALID_OPERATION
*/
int init_wheel(int pi, MotorDriveInfo* target);

/**
 * @brief Drive the wheel forward with specified duty cycle
 *
 * @param pi pigpiod demon handle
 * @param target Target wheel motor driver (e.g WHEELS[0])
 * @param duty Duty cycle (0 to DUTYCYCLE_RANGE)
 * @return RC_OK if OK, otherwise RC_UNINITIALIZED or RC_INVALID_OPERATION
*/
int forward(int pi, const MotorDriveInfo* target, unsigned int duty);

/**
 * @brief Drive the wheel reverse with specified duty cycle
 *
 * @param pi pigpiod demon handle 
 * @param target Target wheel motor driver (e.g WHEELS[0])
 * @param duty Duty cycle (0 to DUTYCYCLE_RANGE)
 * @return RC_OK if OK, otherwise RC_UNINITIALIZED or RC_INVALID_OPERATION
 *
*/
int reverse(int pi, const MotorDriveInfo* target, unsigned int duty);

/**
 * @brief Set the wheel to idle (free-running)
 *
 * @param pi pigpid demon handle
 * @param target Target wheel motor driver
 * @return RC_OK if OK, otherwise RC_UNINITIALIZED or RC_INVALID_OPERATION
*/
int idle(int pi, const MotorDriveInfo* target);

/**
 * @brief Apply on emergency brake to the wheel (short brake)
 *
 * This drives both motor terminals high, causing an abrupt stop
 * Use only emergency stops; frequent use may cause stress on hardware
 *
 * @param pi pigpid demon handle
 * @param target Target wheel motor driver 
 * @return RC_OK if OK, otherwise RC_UNINITIALIZED or RC_INVALID_OPERATION
*/
int brake(int pi, const MotorDriveInfo* target);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //LMP_PROJECT_HARDWARE_MECANUM_WHEEL_CONTROL_H_

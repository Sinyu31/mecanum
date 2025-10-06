#include "mecanum/wheel_control.h"

#ifdef DEBUG 
MotorDriveInfo WHEELS[ROBOT_MANAGED_WHEEL_COUNT] = {
    {.motordrive = {.in1 = FRONT_LEFT_IN1, .in2 = FRONT_LEFT_IN2}, .initialized = false, .index = 0},
    {.motordrive = {.in1 = FRONT_RIGHT_IN1, .in2 = FRONT_RIGHT_IN2}, .initialized = false, .index = 1},
    {.motordrive = {.in1 = REAR_LEFT_IN1, .in2 = REAR_LEFT_IN2}, .initialized = false, .index = 2},
    {.motordrive = {.in1 = REAR_RIGHT_IN1, .in2 = REAR_RIGHT_IN2}, .initialized = false, .index = 3}
};
#else
MotorDriveInfo WHEELS[ROBOT_MANAGED_WHEEL_COUNT] = {
    {.motordrive = {.in1 = FRONT_LEFT_IN1, .in2 = FRONT_LEFT_IN2}, .initialized = false},
    {.motordrive = {.in1 = FRONT_RIGHT_IN1, .in2 = FRONT_RIGHT_IN2}, .initialized = false},
    {.motordrive = {.in1 = REAR_LEFT_IN1, .in2 = REAR_LEFT_IN2}, .initialized = false},
    {.motordrive = {.in1 = REAR_RIGHT_IN1, .in2 = REAR_RIGHT_IN2}, .initialized = false}
};
#endif //DEBUG

static inline unsigned int clamp_upper(unsigned int value, unsigned int upper) {
    return value > upper ? upper : value;
}

static inline int check_init(const MotorDriveInfo* target) {
    if (!target->initialized) {
#ifdef DEBUG
        debug_log(stdout, "[gpio setup warning]: Wheel %s {GPIO (%u, %u)} has not been initialized yet, please call init_wheel() before this function", 
                get_wheel_name(target->index), target->motordrive.in1, target->motordrive.in2);
#endif //DEBUG
       return RC_UNINITIALIZED;

    }
    return RC_OK;
}

static inline int init_wheel_gpio(int pi, const MotorDriveInfo* target) {
   //returns 0 if OK, otherwise PI_BAD_GPIO or PI_BAD_MODE, PI_NOT_PERMITED
    if (set_mode(pi, target->motordrive.in1, PI_OUTPUT) < 0 || set_mode(pi, target->motordrive.in2, PI_OUTPUT) < 0) {
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to set output on Wheel %s {GPIO (%u, %u)} \n", get_wheel_name(target->index), target->motordrive.in1, target->motordrive.in2);
#endif //DEBUG
       return RC_INVALID_OPERATION;
    }
    return RC_OK;
}

static inline int init_wheel_pwm(int pi, const MotorDriveInfo* target) {
    //returns the numerically closest frequency if OK, otherwise PI_BAD_USER_GPIO or PI_NOT_PERMITED
    if (set_PWM_frequency(pi, target->motordrive.in1, FREQUENCY) < 0 || set_PWM_frequency(pi, target->motordrive.in2, FREQUENCY) < 0) {
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to set freq %u on Wheel %s {GPIO (%u, %u)} \n", FREQUENCY, get_wheel_name(target->index), target->motordrive.in1, target->motordrive.in2);
#endif //DEBUG
       return RC_INVALID_OPERATION;
    }
    //returns the real range for the given GPIO's frequency if OK, otherwise PI_BAD_USER_GPIO or PI_BAD_DUTYCYCLE, PI_NOT_PERMITED
    if (set_PWM_range(pi, target->motordrive.in1, DUTYCYCLE_RANGE) < 0 || set_PWM_range(pi, target->motordrive.in2, DUTYCYCLE_RANGE) < 0) { 
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to set range %u on Wheel %s {GPIO (%u, %u)}", DUTYCYCLE_RANGE, get_wheel_name(target->index), target->motordrive.in1, target->motordrive.in2);
#endif //DEBUG
       return RC_INVALID_OPERATION;
    }
    return RC_OK;
}

static inline int write_pwm(int pi, const MotorDriveGPIO* motordrive, unsigned int in1Duty, unsigned int in2Duty) {
    if (set_PWM_dutycycle(pi, motordrive->in1, in1Duty) < 0 || set_PWM_dutycycle(pi, motordrive->in2, in2Duty) < 0) {
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to write pwm to GPIO (%u, %u) \n", motordrive->in1, motordrive->in2);
#endif //DEBUG
        return RC_INVALID_OPERATION;
    }
    return RC_OK;
}

int init_wheel(int pi, MotorDriveInfo* target) {
    assert(target != NULL);
    assert(pi >= 0);

    if (target->initialized) {
#ifdef DEBUG
        debug_log(stdout, "[gpio setup warning]: Wheel %s {GPIO (%u, %u)} is already initialized", get_wheel_name(target->index), target->motordrive.in1, target->motordrive.in2);
#endif //DEBUG
       return RC_ALREADY_INITIALIZED;
    }

    int rc;
    rc = init_wheel_gpio(pi, target);
    if (rc != RC_OK) {
        return rc;
    }
    rc = init_wheel_pwm(pi, target);
    if (rc != RC_OK) {
        return rc;
    }

    target->initialized = true;
    return write_pwm(pi, &target->motordrive, 0, 0);
}

int forward(int pi, const MotorDriveInfo* target, unsigned int duty) {
    assert(target != NULL);
    assert(pi >= 0);

    if (unlikely(check_init(target) != RC_OK)) {
        return RC_UNINITIALIZED;
    }
    
    duty = clamp_upper(duty, DUTYCYCLE_RANGE);
    return write_pwm(pi, &target->motordrive, duty, 0);
}

int reverse(int pi, const MotorDriveInfo* target, unsigned int duty) {
    assert(target != NULL);
    assert(pi >= 0);

    if (unlikely(check_init(target) != RC_OK)) {
        return RC_UNINITIALIZED;
    }

    duty = clamp_upper(duty, DUTYCYCLE_RANGE);
    return write_pwm(pi, &target->motordrive, 0, duty);

}

int idle(int pi, const MotorDriveInfo* target) {
    assert(target != NULL);
    assert(pi >= 0);

    if (unlikely(check_init(target) != RC_OK)) {
        return RC_UNINITIALIZED;
    }

    return write_pwm(pi, &target->motordrive, 0, 0);
}

int brake(int pi, const MotorDriveInfo* target) {
    assert(target != NULL);
    assert(pi >= 0);

    if (unlikely(check_init(target) != RC_OK)) {
        return RC_UNINITIALIZED;
    }

    return write_pwm(pi, &target->motordrive, DUTYCYCLE_RANGE, DUTYCYCLE_RANGE);
}

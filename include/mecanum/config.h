#ifndef LMP_PROJECT_HARDWARE_MECANUM_CONFIG_H_
#define LMP_PROJECT_HARDWARE_MECANUM_CONFIG_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <pigpiod_if2.h>
#include <assert.h>
#include <time.h>

#ifdef DEBUG
    #include "mecanum/debug.h"
#endif //DEBUG

#define NUM_WIRES_PER_WHEEL 2U
#define ROBOT_MANAGED_WHEEL_COUNT 4U

#define GPIO_UNASSIGNED 0U

#define HIGH 1
#define LOW 0

#define RC_OK 0
#define RC_ALREADY_INITIALIZED 1
#define RC_INVALID_OPERATION -1
#define RC_UNINITIALIZED -2
#define RC_UNKNOWN_MODE -3
#define RC_FAIL_DAEMON_CONNECT -4

#define LOCALHOST NULL
#define DEFAULT_PORT NULL

#if defined(__GNUC__) || defined(__clang__)
  #define likely(x) __builtin_expect(!!x, 1)
  #define unlikely(x) __builtin_expect(!!x, 0)
#else 
  #define likely(x) !!(x)
  #define unlikely(x) !!(x)
#endif

#define UNUSED_PARAMETER(x) (void)x

#endif //LMP_PROJECT_HARDWARE_MECANUM_CONFIG_H_

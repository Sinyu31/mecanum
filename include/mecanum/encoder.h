#ifndef LMP_PROJECT_HARDWARE_MECANUM_ENCODER_H_
#define LMP_PROJECT_HARDWARE_MECANUM_ENCODER_H_

#include "mecanum/config.h"

#define ENCODER_FRONT_LEFT_CH_A GPIO_UNASSIGNED
#define ENCODER_FRONT_LEFT_CH_B GPIO_UNASSIGNED

#define ENCODER_FRONT_RIGHT_CH_A GPIO_UNASSIGNED
#define ENCODER_FRONT_RIGHT_CH_B GPIO_UNASSIGNED

#define ENCODER_REAR_LEFT_CH_A GPIO_UNASSIGNED
#define ENCODER_REAR_LEFT_CH_B GPIO_UNASSIGNED

#define ENCODER_REAR_RIGHT_CH_A GPIO_UNASSIGNED
#define ENCODER_REAR_RIGHT_CH_B GPIO_UNASSIGNED

#define MASK_LOWER2 0x3

#define MIN_PULSE_US 50

typedef enum {UNSET = 0, X1 = 1 , X2 = 2, X4 = 4} EncoderMultipication;

typedef struct {
    unsigned int cha;
    unsigned int chb;
} EncoderGPIO;

#ifdef DEBUG
typedef struct {
    const EncoderGPIO encoder;
    EncoderMultipication mode;
    volatile int32_t position;
    volatile uint32_t tick;
    uint8_t prevState; // store level A in bit 1, store level B in bit 0
    bool initialized;
    const uint8_t index;
} EncoderInfo;

extern EncoderInfo ENCODERS[ROBOT_MANAGED_WHEEL_COUNT];
#else
typedef struct {
    const EncoderGPIO encoder;
    EncoderMultipication mode;
    volatile int32_t position;
    volatile uint32_t tick;
    uint8_t prevState;
    bool initialized;
} EncoderInfo;

extern EncoderInfo ENCODERS[ROBOT_MANAGED_WHEEL_COUNT];
#endif //DEBUG

#ifdef  __cplusplus
extern "C" {
#endif //__cplusplus

int init_encoder(EncoderInfo* target, EncoderMultipication mode);

int deinit_encoder(EncoderInfo* target, bool clearMember);

int32_t get_position(const EncoderInfo* target);

void set_position(EncoderInfo* target, int32_t val);
#ifdef __cplusplus
}
#endif //__cplusplus

#endif //LMP_PROJECT_HARDWARE_MECANUM_ENCODER_H_ 

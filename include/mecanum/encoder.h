#ifndef LMP_PROJECT_HARDWARE_MECANUM_ENCODER_H_
#define LMP_PROJECT_HARDWARE_MECANUM_ENCODER_H_

#include "mecanum/config.h"

/**
 * @file encoder.h
 * @brief Use the encoder attached to the DC motor to obtain the position
 *
 * Encoders are used to measure wheel rotation, direction and angular velocity
 * Each encoder has two channels (A and B)
*/

/* Constants */
#define MASK_LOWER2 0x3
#define MIN_PULSE_US 50  //Use it to avoid chattering

/* GPIO Configuration */
#define ENCODER_FRONT_LEFT_CH_A GPIO_UNASSIGNED
#define ENCODER_FRONT_LEFT_CH_B GPIO_UNASSIGNED

#define ENCODER_FRONT_RIGHT_CH_A GPIO_UNASSIGNED
#define ENCODER_FRONT_RIGHT_CH_B GPIO_UNASSIGNED

#define ENCODER_REAR_LEFT_CH_A GPIO_UNASSIGNED
#define ENCODER_REAR_LEFT_CH_B GPIO_UNASSIGNED

#define ENCODER_REAR_RIGHT_CH_A GPIO_UNASSIGNED
#define ENCODER_REAR_RIGHT_CH_B GPIO_UNASSIGNED

/**
 * @enum EncoderMultiplication
 * @brief Specifies the multiplication mode for the encoder
 *
 * -X1: count on rising edge (or falling edge) of one channel
 * -X2: count on both edge of one channel
 * -X4: count on both edge of both channels
*/
typedef enum {UNSET = 0, X1 = 1 , X2 = 2, X4 = 4} EncoderMultiplication;

/**
 * @struct EncoderGPIO
 * @brief Holds encoder input pin numbers 
*/
typedef struct {
    unsigned int cha; //channel A
    unsigned int chb; //channel B
} EncoderGPIO;

#ifdef DEBUG
/**
 * @struct EncoderInfo
 * @brief Encoder information with debug index
*/
typedef struct {
    const EncoderGPIO encoder;  //encoder pins
    EncoderMultiplication mode; //Multiplication mode (X1, X2, or X4)  
    volatile int32_t position;  //Accumulated position
    volatile uint32_t tick;     //Timestamp of last tick (Internal use only)
    uint8_t prevState;          //Previous status (bit1 = A, bit0 = B) (Internal use only)
    bool initialized;           //Initialization status
    const uint8_t index;        //Encoder index (use debug)
} EncoderInfo;

extern EncoderInfo ENCODERS[ROBOT_MANAGED_WHEEL_COUNT];
#else
/**
 * @struct EncoderInfo
 * @brief Encoder information (release version)
 *
*/
typedef struct {
    const EncoderGPIO encoder;  //encoder pins
    EncoderMultiplication mode; //Multiplication mode (X1, X2, or X4)
    volatile int32_t position;  //Accumulated position
    volatile uint32_t tick;     //Timestamp of last tick (Internal use only)
    uint8_t prevState;          //Previous status (bit1 = A, bit0 = B) (Internal use only)
    bool initialized;           //Initialization status
} EncoderInfo;

extern EncoderInfo ENCODERS[ROBOT_MANAGED_WHEEL_COUNT];
#endif //DEBUG

#ifdef  __cplusplus
extern "C" {
#endif //__cplusplus

/**
 * @brief Initialize an encoder with given multiplication mode 
 *
 * @param target Target encoder (e.g ENCODERS[0])
 * @param mode Multiplication mode (X1, X2, or X4)
 * @return RC_OK if OK, otherwise RC_ALREADY_INITIALIZED or RC_UNKNOWN_MODE or RC_INVALID_OPERATION
*/
int init_encoder(EncoderInfo* target, EncoderMultiplication mode);

/**
 * @brief Deinitialize an encoder
 * 
 * @param target Target encoder (e.g ENCODERS[0])
 * @param clearMember If true, reset position and state fields
 * @return RC_OK if OK, otherwise RC_UNINITIALIZED;
*/
int deinit_encoder(EncoderInfo* target, bool clearMember);

/**
 * @brief Get the current position of an encoder
 *
 * @param target Target encoder (e.g ENCODERS[0])
 * @return Current position
*/
int32_t get_position(const EncoderInfo* target);

/**
 * @brief Manually set the position of an encoder
 *
 * @param target Target Encoder (e.g ENCODERS[0])
 * @param val new position
*/
void set_position(EncoderInfo* target, int32_t val);
#ifdef __cplusplus
}
#endif //__cplusplus

#endif //LMP_PROJECT_HARDWARE_MECANUM_ENCODER_H_ 

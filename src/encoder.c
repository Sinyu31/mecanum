#include "mecanum/encoder.h"
#include <assert.h>

/**      
 * defined the state as follows: (if the lower 2 bits are AB)
 *   state 0: 00     | index 0
 *   state 1: 01     | index 1
 *   state 2: 11     | index 3
 *   state 3: 10     | index 2
 *
 *
 *  [cw]   state 0 -> state 3 -> state 2 -> state 1 -> state 0 -> ...
 *         index 0 -> index 2 -> index 3 -> index 1 -> idnex 0 -> ...
 *
 *      1         _____       _____       _____       _____
 *  A            |     |     |     |     |     |     |     |                ch A rising: ch B is LOW
 *      0   _____|     |_____|     |_____|     |_____|     |_____           ch A falling: ch B is HIGH
 *                         
 *      ------------------------------------------------------------> t
 *
 *      1   ___       _____       _____       _____       _____
 *  B          |     |     |     |     |     |     |     |     |            ch B rising: ch A is HIGH
 *      0      |_____|     |_____|     |_____|     |_____|     |_           ch B falling: ch A is Low

 * 
 *
 *   [ccw]   state 0 -> state 1 -> state 2 -> state 3 -> state 0 -> ...
 *           index 0 -> index 1 -> index 3 -> index 2 -> index 0 -> ...
 *  
 *       1 ___       _____       _____       _____       _____
 *  A         |     |     |     |     |     |     |     |     |            ch A rising: ch B is HIGH
 *       0    |_____|     |_____|     |_____|     |_____|     |_           ch A falling: ch B is LOW
 * 
 *       ----------------------------------------------------------->  t  
 *
 *       1        _____       _____       _____       _____
 *  B            |     |     |     |     |     |     |     |               ch B rising: ch A is LOW
 *       0  _____|     |_____|     |_____|     |_____|     |_____          ch B falling: ch A is HIGH
 *   
 *
**/

#ifdef DEBUG
EncoderInfo ENCODERS[ROBOT_MANAGED_WHEEL_COUNT] = {
    {.encoder = {.cha = ENCODER_FRONT_LEFT_CH_A, .chb = ENCODER_FRONT_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false, .index = 0},
    {.encoder = {.cha = ENCODER_FRONT_RIGHT_CH_A, .chb = ENCODER_FRONT_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false, .index = 1},
    {.encoder = {.cha = ENCODER_REAR_LEFT_CH_A, .chb = ENCODER_REAR_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false, .index = 2},
    {.encoder = {.cha = ENCODER_REAR_RIGHT_CH_A, .chb = ENCODER_REAR_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false, .index = 3}
};
#else
EncoderInfo ENCODERS[ROBOT_MANAGED_WHEEL_COUNT] = {
    {.encoder = {.cha = ENCODER_FRONT_LEFT_CH_A, .chb = ENCODER_FRONT_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false},
    {.encoder = {.cha = ENCODER_FRONT_RIGHT_CH_A, .chb = ENCODER_FRONT_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false},
    {.encoder = {.cha = ENCODER_REAR_LEFT_CH_A, .chb = ENCODER_REAR_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false},
    {.encoder = {.cha = ENCODER_REAR_RIGHT_CH_A, .chb = ENCODER_REAR_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .prevState = 0x0, .initialized = false}
};
#endif //DEBUG

static void on_edge_changed_x1(int gpio, int level, uint32_t tick, void* userdata) {
    assert(userdata != NULL);
    EncoderInfo* ei = (EncoderInfo*)userdata;
    
    assert(gpio == ei->encoder.cha);
    if (tick - ei->tick < MIN_PULSE_US) return;
    
    //There is always an interruption when the edge is standing, so just check at B
    ei->position += gpioRead(ei->encoder.chb) == LOW ? 1 : -1;
    ei->tick = tick;
}

static void on_edge_changed_x2(int gpio, int level, uint32_t tick, void* userdata) {
    assert(userdata != NULL);
    EncoderInfo* ei = (EncoderInfo*)userdata;

    assert(gpio == ei->encoder.cha);
    if (tick - ei->tick < MIN_PULSE_US) return;
    
    static const int8_t LOOKUP_X2[2][2] = {{-1, 1}, {1, -1}};
        
    int levelA = level;
    int levelB = gpioRead(ei->encoder.chb);

    ei->position += LOOKUP_X2[levelA][levelB];
    ei->tick = tick;
}

static void on_edge_changed_x4(int gpio, int level, uint32_t tick, void* userdata) {
    assert(userdata != NULL);
    EncoderInfo* ei = (EncoderInfo*)userdata;

    if (tick - ei->tick < MIN_PULSE_US) return;

    static const int8_t LOOKUP_X4[4][4] = {
        {0, -1, 1, 0},
        {1, 0, 0, -1},
        {-1, 0, 0, 1},
        {0, 1, -1, 0}
    };

    int levelA, levelB;

    if (gpio == ei->encoder.cha) {
        levelA = level;
        levelB = gpioRead(ei->encoder.chb);
    }
    else {
        levelA = gpioRead(ei->encoder.cha);
        levelB = level;
    }

    int currentState = (levelA << 1) | levelB;
    int prevState = ei->prevState;
    
    ei->position += LOOKUP_X4[prevState][currentState];
    ei->prevState = currentState & MASK_LOWER2;
    ei->tick = tick;
}  

static inline int init_encoder_gpio(const EncoderInfo* target) {
    unsigned int cha = target->encoder.cha;
    unsigned int chb = target->encoder.chb;   

    //returns 0 if OK, otherwise PI_BAD_GPIO or PI_BAD_MODE
    if (gpioSetMode(cha, PI_INPUT) < 0 || gpioSetMode(chb, PI_INPUT) < 0) {
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to set input on Encoder %s {GPIO (%u, %u)} \n", get_encoder_name(target->index), cha, chb);
#endif //DEBUG
       return RC_INVALID_OPERATION;
    } 
    //returns 0 if OK, otherwise PI_BAD_GPIO or PI_BAD_PUD
    if (gpioSetPullUpDown(cha, PI_PUD_UP) < 0 || gpioSetPullUpDown(chb, PI_PUD_UP) < 0) {
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to set pull up on Encoder %s {GPIO (%u, %u)} \n", get_encoder_name(target->index), cha, chb);
#endif //DEBUG
       return RC_INVALID_OPERATION; 
    } 
    return RC_OK;
 } 
 
int init_encoder(EncoderInfo* target, EncoderMultiplication mode) {
   assert(target != NULL);
  
    if (target->initialized) {
#ifdef DEBUG
        debug_log(stdout, "[gpio setup warning]: Encoder %s {GPIO (%u, %u)} is already initialized \n", get_encoder_name(target->index), target->encoder.cha, target->encoder.chb);
#endif //DEBUG
        return RC_ALREADY_INITIALIZED; 
    }
    if (init_encoder_gpio(target) != RC_OK) {
        return RC_INVALID_OPERATION;
    } 

    int levelA = gpioRead(target->encoder.cha);
    int levelB = gpioRead(target->encoder.chb);
    target->prevState = (levelA << 1) | levelB;

    switch (mode) {
        case X1:
            if (gpioSetISRFuncEx(target->encoder.cha, RISING_EDGE, 0, on_edge_changed_x1, target) < 0) {
#ifdef DEBUG
                debug_log(stderr, "[gpio invalid operation error]: Failed to register interrunpt on Encoder %s {GPIO (%u)} \n", get_encoder_name(target->index), target->encoder.cha);
#endif //DEBUG
                return RC_INVALID_OPERATION;
            } 
            break;

        case X2:
            if (gpioSetISRFuncEx(target->encoder.cha, EITHER_EDGE, 0, on_edge_changed_x2, target) < 0) {
#ifdef DEBUG
                debug_log(stderr, "[gpio invalid operation error]: Failed to register interrupt on Encoder %s {GPIO (%u)} \n", get_encoder_name(target->index), target->encoder.cha);
#endif //DEBUG
                return RC_INVALID_OPERATION;
            }
            break;

        case X4:
            if (gpioSetISRFuncEx(target->encoder.cha, EITHER_EDGE, 0, on_edge_changed_x4, target) < 0 || gpioSetISRFuncEx(target->encoder.chb, EITHER_EDGE, 0, on_edge_changed_x4, target) < 0) {
#ifdef DEBUG
                debug_log(stderr, "[gpio invalid operation error]: Failed to register interrupt on Encoder %s {GPIO (%u, %u)} \n", get_encoder_name(target->index), target->encoder.cha, target->encoder.chb);
#endif //DEBUG
                return RC_INVALID_OPERATION;
            }
            break;

        default:
#ifdef DEBUG 
#endif //DEBUG
       return RC_UNKNOWN_MODE;
    }

    target->mode = mode;
    target->initialized = true;
     return RC_OK;      
}  

int deinit_encoder(EncoderInfo* target, bool cleared) {
    assert(target != NULL);
    if (!target->initialized) {
#ifdef DEBUG
        debug_log(stderr, "[gpio setup warning]: Encoder %s {GPIO (%u, %u}) has not been initialized yet, please call init_encoder() before this function", get_encoder_name(target->index), target->encoder.cha, target->encoder.chb);
#endif //DEBUG
       return RC_UNINITIALIZED;
    }
    switch (target->mode) {
    case X1:
        (void)gpioSetISRFuncEx(target->encoder.cha, RISING_EDGE, 0, NULL, NULL);
        break;
    case X2:
        (void)gpioSetISRFuncEx(target->encoder.cha, EITHER_EDGE, 0, NULL, NULL);
        break;
    case X4:
        (void)gpioSetISRFuncEx(target->encoder.cha, EITHER_EDGE, 0, NULL, NULL);
        (void)gpioSetISRFuncEx(target->encoder.chb, EITHER_EDGE, 0, NULL, NULL);
        break;
    default:
        break;
    }
    target->initialized = false;
    target->mode = UNSET;
    if (cleared) {
        target->position = 0;
        target->tick = 0;
        target->prevState = 0;
    }
    return RC_OK;
}

int32_t get_position(const EncoderInfo* target) {
    assert(target != NULL);
    return target->position;
}

void set_position(EncoderInfo* target, int32_t val){
    assert(target != NULL);
    target->position = val;
}

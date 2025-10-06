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
    {.encoder = {.cha = ENCODER_FRONT_LEFT_CH_A, .chb = ENCODER_FRONT_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1,  .prevState = 0x0, .initialized = false, .index = 0},
    {.encoder = {.cha = ENCODER_FRONT_RIGHT_CH_A, .chb = ENCODER_FRONT_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1, .prevState = 0x0, .initialized = false, .index = 1},
    {.encoder = {.cha = ENCODER_REAR_LEFT_CH_A, .chb = ENCODER_REAR_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1, .prevState = 0x0, .initialized = false, .index = 2},
    {.encoder = {.cha = ENCODER_REAR_RIGHT_CH_A, .chb = ENCODER_REAR_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1, .prevState = 0x0, .initialized = false, .index = 3}
};
#else
EncoderInfo ENCODERS[ROBOT_MANAGED_WHEEL_COUNT] = {
    {.encoder = {.cha = ENCODER_FRONT_LEFT_CH_A, .chb = ENCODER_FRONT_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1, .prevState = 0x0, .initialized = false},
    {.encoder = {.cha = ENCODER_FRONT_RIGHT_CH_A, .chb = ENCODER_FRONT_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1, .prevState = 0x0, .initialized = false},
    {.encoder = {.cha = ENCODER_REAR_LEFT_CH_A, .chb = ENCODER_REAR_LEFT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1, .prevState = 0x0, .initialized = false},
    {.encoder = {.cha = ENCODER_REAR_RIGHT_CH_A, .chb = ENCODER_REAR_RIGHT_CH_B}, .mode = UNSET, .position = 0, .tick = 0, .callback_id_a = -1, .callback_id_b = -1, .prevState = 0x0, .initialized = false}
};
#endif //DEBUG

static void on_edge_changed_x1(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void* userdata) {
    assert(userdata != NULL);
    UNUSED_PARAMETER(level);
    EncoderInfo* ei = (EncoderInfo*)userdata;
    
    assert(gpio == ei->encoder.cha);
    if ((uint32_t)(tick - ei->tick) < MIN_PULSE_US) return;
    
    //There is always an interruption when the edge is standing, so just check at B
    ei->position += gpio_read(pi, ei->encoder.chb) == LOW ? 1 : -1;
    ei->tick = tick;
}

static void on_edge_changed_x2(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void* userdata) {
    assert(userdata != NULL);
    EncoderInfo* ei = (EncoderInfo*)userdata;

    assert(gpio == ei->encoder.cha);
    if ((uint32_t)(tick - ei->tick) < MIN_PULSE_US) return;
    
    static const int8_t LOOKUP_X2[2][2] = {{-1, 1}, {1, -1}};
        
    int levelA = level;
    int levelB = gpio_read(pi, ei->encoder.chb);

    ei->position += LOOKUP_X2[levelA][levelB];
    ei->tick = tick;
}

static void on_edge_changed_x4(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void* userdata) {
    assert(userdata != NULL);
    EncoderInfo* ei = (EncoderInfo*)userdata;

	if ((uint32_t)(tick - ei->tick) < MIN_PULSE_US) return;

    static const int8_t LOOKUP_X4[4][4] = {
        {0, -1, 1, 0},
        {1, 0, 0, -1},
        {-1, 0, 0, 1},
        {0, 1, -1, 0}
    };

    int levelA, levelB;

    if (gpio == ei->encoder.cha) {
        levelA = level;
        levelB = gpio_read(pi, ei->encoder.chb);
    }
    else {
        levelA = gpio_read(pi, ei->encoder.cha);
        levelB = level;
    }

    int currentState = ((levelA << 1) | levelB) & MASK_LOWER2;
    int prevState = ei->prevState;
    
    ei->position += LOOKUP_X4[prevState][currentState];
    ei->prevState = currentState & MASK_LOWER2;
    ei->tick = tick;
}  

static inline int init_encoder_gpio(int pi, const EncoderInfo* target) {
    unsigned int cha = target->encoder.cha;
    unsigned int chb = target->encoder.chb;   

    //returns 0 if OK, otherwise PI_BAD_GPIO or PI_BAD_MODE, PI_NOT_PREMITED
    if (set_mode(pi, cha, PI_INPUT) < 0 || set_mode(pi, chb, PI_INPUT) < 0) {
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to set input on Encoder %s {GPIO (%u, %u)} \n", get_encoder_name(target->index), cha, chb);
#endif //DEBUG
       return RC_INVALID_OPERATION;
    } 
    //returns 0 if OK, otherwise PI_BAD_GPIO or PI_BAD_PUD, PI_NOT_PREMITED
    if (set_pull_up_down(pi, cha, PI_PUD_UP) < 0 || set_pull_up_down(pi, chb, PI_PUD_UP) < 0) {
#ifdef DEBUG
        debug_log(stderr, "[gpio invalid operation error]: Failed to set pull up on Encoder %s {GPIO (%u, %u)} \n", get_encoder_name(target->index), cha, chb);
#endif //DEBUG
       return RC_INVALID_OPERATION; 
    } 
    return RC_OK;
 } 
 
int init_encoder(int pi, EncoderInfo* target, EncoderMultiplication mode) {
    assert(target != NULL);
    assert(pi >= 0);

    if (target->initialized) {
#ifdef DEBUG
        debug_log(stdout, "[gpio sietup warning]: Encoder %s {GPIO (%u, %u)} is already initialized \n", get_encoder_name(target->index), target->encoder.cha, target->encoder.chb);
#endif //DEBUG
        return RC_ALREADY_INITIALIZED; 
    }
    if (init_encoder_gpio(pi, target) != RC_OK) {
        return RC_INVALID_OPERATION;
    } 

    int levelA = gpio_read(pi, target->encoder.cha);
    int levelB = gpio_read(pi, target->encoder.chb);
    target->prevState = ((levelA << 1) | levelB) & MASK_LOWER2;

    switch (mode) {
        case X1:
            target->callback_id_a = callback_ex(pi, target->encoder.cha, RISING_EDGE, on_edge_changed_x1, target);
            if (target->callback_id_a < 0) {
#ifdef DEBUG
                debug_log(stderr, "[gpio invalid operation error]: Failed to register interrunpt on Encoder %s {GPIO (%u)} \n", get_encoder_name(target->index), target->encoder.cha);
#endif //DEBUG
                return RC_INVALID_OPERATION;
            } 
            break;

        case X2:
            target->callback_id_a = callback_ex(pi, target->encoder.cha, EITHER_EDGE, on_edge_changed_x2, target);
            if (target->callback_id_a < 0) {
#ifdef DEBUG
                debug_log(stderr, "[gpio invalid operation error]: Failed to register interrupt on Encoder %s {GPIO (%u)} \n", get_encoder_name(target->index), target->encoder.cha);
#endif //DEBUG
                return RC_INVALID_OPERATION;
            }
            break;

        case X4:
            target->callback_id_a = callback_ex(pi, target->encoder.cha, EITHER_EDGE, on_edge_changed_x4, target);
            target->callback_id_b = callback_ex(pi, target->encoder.chb, EITHER_EDGE, on_edge_changed_x4, target);
            if (target->callback_id_a < 0 || target->callback_id_b < 0) {
#ifdef DEBUG
                debug_log(stderr, "[gpio invalid operation error]: Failed to register interrupt on Encoder %s {GPIO (%u, %u)} \n", get_encoder_name(target->index), target->encoder.cha, target->encoder.chb);
#endif //DEBUG
                return RC_INVALID_OPERATION;
            }
            break;

        default:
       return RC_UNKNOWN_MODE;
    }

    target->mode = mode;
    target->initialized = true;
     return RC_OK;      
}  

int deinit_encoder(int pi, EncoderInfo* target, bool cleared) {
    assert(target != NULL);
    assert(pi >= 0);

    if (!target->initialized) {
#ifdef DEBUG
        debug_log(stderr, "[gpio setup warning]: Encoder %s {GPIO (%u, %u}) has not been initialized yet, please call init_encoder() before this function", get_encoder_name(target->index), target->encoder.cha, target->encoder.chb);
#endif //DEBUG
       return RC_UNINITIALIZED;
    }
    switch (target->mode) {
    case X1:
        if (target->callback_id_a >= 0)
            (void)callback_cancel((unsigned int)target->callback_id_a);
        break;
    case X2:
        if (target->callback_id_a >= 0)
            (void)callback_cancel((unsigned int)target->callback_id_a);
        break;
    case X4:
        if (target->callback_id_a >= 0)
            (void)callback_cancel((unsigned int)target->callback_id_a);
        if (target->callback_id_b >= 0) 
            (void)callback_cancel((unsigned int)target->callback_id_b);
        break;
    default:
        break;
    }
    target->initialized = false;
    target->mode = UNSET;
    target->callback_id_a = -1;
    target->callback_id_b = -1;

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

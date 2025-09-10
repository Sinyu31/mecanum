#ifndef LMP_PROJECT_HARDWARE_MECANUM_DEBUG_H_
#define LMP_PROJECT_HARDWARE_MECANUM_DEBUG_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

static inline const char* get_wheel_name(uint8_t bit) {
	switch (bit) {
		    case 0:
		        return "FL";
		    case 1:
		        return "FR";
		    case 2:
		        return "RL";
		    case 3:
		        return "RR";
		    default:
		        return NULL;
		}

}

static inline const char* get_encoder_name(uint8_t bit) {
	switch (bit) {
        case 0:
            return "EFL";
        case 1:
            return "EFR";
        case 2:
            return "ERL";
        case 3:
            return "ERR";
        default:
            return NULL;
    }
}

static inline void debug_log(FILE* fp, const char* fmt, ...) {
	va_list args;
    va_start(args, fmt);
    (void)vfprintf(fp, fmt, args);
    va_end(args);
    (void)fflush(fp);
}

#endif //LMP_PROJECT_HARDWARE_MECANUM_DEBUG_H_

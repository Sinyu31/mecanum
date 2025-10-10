#include "mecanum/daemon.h"

int pigpiod_daemon_open(const char* addr, const char* port) {
        int pi = pigpio_start(addr, port);
        if (pi >= 0) return pi;
#ifdef DEBUG
    debug_log(stderr, "[pigpiod daemon]: Failed to connect daemon \n");
#endif //DEBUG
       return RC_FAIL_DAEMON_CONNECT;
}

void pigpiod_daemon_close(int pi) {
    assert(pi >= 0);
    pigpio_stop(pi);
}

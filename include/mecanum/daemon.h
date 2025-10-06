#ifndef LMP_PROJECT_HARDWARE_MECANUM_DAEMON_H_
#define LMP_PROJECT_HARDWARE_MECANUM_DAEMON_H_

#include "mecanum/config.h"

/**
 * @file daemon.h
 * @brief Provides connection management to the pigpiod daemon
 *
*/


#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
 * @brief Connect pigpiod daemon
 *
 * @param addr Hostname or IP address of the Pi running the pigpiod daemon.
 *             Pass NULL or LOCALHOST to use local machine.
 * @param port TCP port of the pigpiod daemon
 *             Pass NULL or DEFAULT_PORT for the default port.
 *
 * @return >= 0 if OK (pigpiod handle), otherwise RC_FAIL_DAEMON_CONNECT 
*/
int pigpiod_daemon_open(const char* addr, const char* port);

/**
 * @brief Disconnect form the pigpiod daemon
 *
 * @param handle The handle returned by pigpiod_daemon_open().
*/
void pigpiod_daemon_close(int handle);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //LMP_PROJECT_HARDWARE_MECANUM_DAEMON_H_

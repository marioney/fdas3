/**
 * Common utilities for the FDAS3 devices.
 */

#ifndef UTILS_H
#define UTILS_H


#include <stdint.h>
#include <time.h>


/**
 * Get current time in microseconds since epoch.
 */
static inline uint64_t get_time_us() {
    struct timespec t;
    if (clock_gettime(CLOCK_REALTIME, &t)) {
        syslog(LOG_ERR, "Error getting time: %s", strerror(errno));
        return 0;
    }
    
    return (uint64_t)t.tv_sec * 1000 + t.tv_nsec / 1000;
}



#endif//UTILS_H

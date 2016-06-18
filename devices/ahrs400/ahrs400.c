/**
 * Device module for Crossbow's AHRS400 Attitude and Heading Reference System.
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <syslog.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>


#include "ahrs400.h"


/*** AHRS constants ***/
#define AHRS_GYRO_RANGE (200 * M_PI / 180)
#define AHRS_G_RANGE 4
#define AHRS_DEFAULT_BAUDRATE B38400

#define AHRS_DATA_HEADER 0xFF
#define AHRS_MAX_MSG_SIZE 30

/*** AHRS Message codes ***/
// Communication test messages
#define PING 'R'
#define PING_RESPONSE 'H'

// Measurement mode configuration messages
#define VOLTAGE_MODE 'r'
#define VOLTAGE_MODE_RESPONSE 'R'
#define SCALED_MODE 'c'
#define SCALED_MODE_RESPONSE 'C'
#define ANGLE_MODE 'a'
#define ANGLE_MODE_RESPONSE 'A'

// Communication mode configuration messages
#define POLLED_MODE 'P'
#define CONTINUOUS_MODE 'C'
#define REQUEST_DATA 'G'
#define REQUEST_BAUD 'b'
#define REQUEST_BAUD_RESPONSE 'B'
#define NEW_BAUD 'a'
#define NEW_BAUD_RESPONSE 'A'

// Information query messages
#define QUERY_VERSION 'v'
#define QUERY_VERSION_LENGTH 26
#define QUERY_SERIAL_NUMBER 'S'

// Magnetic calibration messages
#define START_CALIB 's'
#define START_CALIB_RESPONSE 'S'
#define END_CALIB 'u'
#define END_CALIB_RESPONSE 'U'
#define CLEAR_HARDI 'h'
#define CLEAR_HARDI_RESPONSE 'H'
#define CLEAR_SOFTI 't'
#define CLEAR_SOFTI_RESPONSE 'T'


/**
 * Open the AHRS serial port stream.
 * @param The path of the serial port device.
 * @return The AHRS port stream or NULL if error.
 */
FILE* ahrs_open(char *path) {
    int fd = open(path, O_RDWR);
    if (fd < 0) {
        char *msg = "Error opening AHRS port `%s`: %s";
        syslog(LOG_ERR, msg, path, strerror(errno));
        return NULL;
    }
    
    FILE *file = fdopen(fd, "r+b");
    if (!file) {
        syslog(LOG_ERR, "Error in fdopen: %s", strerror(errno));
        close(fd);
        return NULL;
    }
    
    struct termios ahrs_termios;
    if (tcgetattr(fd, &ahrs_termios)
        || cfsetispeed(&ahrs_termios, AHRS_DEFAULT_BAUDRATE)
        || cfsetospeed(&ahrs_termios, AHRS_DEFAULT_BAUDRATE)
        || tcsetattr(fd, TCSANOW, &ahrs_termios)) {
        char *msg = "Error setting AHRS serial port baud rate: %s";
        syslog(LOG_WARNING, msg, strerror(errno));
    } else {
        cfmakeraw(&ahrs_termios);
        if (tcsetattr(fd, TCSANOW, &ahrs_termios)) {
            char *msg = "Error making AHRS serial port raw: %s";
            syslog(LOG_WARNING, msg, strerror(errno));            
        }
    }
    
    return file;
}


/**
 * Ping the AHRS.
 * @param AHRS400 serial port stream.
 * @return 0 if pong received, -1 if pong not received, if error or if EOF.
 */
int ahrs_ping(FILE *file) {
    if (fputc(PING, file) == EOF) {
        syslog(LOG_ERR, "Error writing ping to AHRS stream: %s",
               strerror(errno));
        return -1;
    }

    int response = fgetc(file);
    if (response == EOF) {
        if (feof(file))
            syslog(LOG_WARNING, "EOF while waiting for ping response");
        else
            syslog(LOG_ERR, "Read error while waiting for ping response: %s",
                   strerror(errno));
        return -1;
    }
    
    if (response != PING_RESPONSE) {
        syslog(LOG_INFO, "Invalid ping from AHRS: %#x", response);
        return -1;
    }

    return 0;
}


/**
 * Put the AHRS in the continuous mode.
 * @param AHRS400 serial port stream.
 * @return 0 if success, -1 if error.
 */
int ahrs_set_continuous(FILE *file) {
    if (fputc(CONTINUOUS_MODE, file) == EOF) {
        syslog(LOG_ERR, "Error writing continuous mode to AHRS stream: %s",
               strerror(errno));
        return -1;
    }
    return 0;    
}


/**
 * Put the AHRS in the polled mode.
 * @param AHRS400 serial port stream.
 * @return 0 if success, -1 if error.
 */
int ahrs_set_polled(FILE *file) {
    if (fputc(POLLED_MODE, file) == EOF) {
        syslog(LOG_ERR, "Error writing polled mode to AHRS stream: %s",
               strerror(errno));
        return -1;
    }
    return 0;
}


/**
 * Flush the AHRS input/output stream.
 * @param AHRS400 serial port stream.
 * @return 0 if success, -1 if error.
 */
int ahrs_purge(FILE *file) {
    int fd = fileno(file);
    if (fd < 0) {
        syslog(LOG_ERR, "Error getting file descriptor: %s", strerror(errno));
        return -1;
    }
    
    if (tcflush(fd, TCIOFLUSH)) {
        syslog(LOG_WARNING, "Error flushing stream: %s", strerror(errno));
        return -1;
    }
    
    return 0;
}



/**
 * Set the AHRS measurement mode.
 * @param AHRS400 serial port stream.
 * @param desired mode.
 * @return 0 if success received, -1 if error or EOF.
 */
int ahrs_set_mode(FILE *file, ahrs_mode_t mode) {
    char mode_command, mode_response;
    
    switch (mode) {
    case AHRS_VOLTAGE_MODE:
        mode_command = VOLTAGE_MODE;
        mode_response = VOLTAGE_MODE_RESPONSE;
        break;
    case AHRS_SCALED_MODE:
        mode_command = SCALED_MODE;
        mode_response = SCALED_MODE_RESPONSE;
        break;
    case AHRS_ANGLE_MODE:
        mode_command = ANGLE_MODE;
        mode_response = ANGLE_MODE_RESPONSE;
        break;
    default:
        syslog(LOG_ERR, "Unknown AHRS mode");
        return -1;
    }
    
    if (fputc(mode_command, file) == EOF) {
        syslog(LOG_ERR, "Error writing mode command to AHRS stream: %s",
               strerror(errno));
        return -1;
    }
    
    int response = fgetc(file);
    if (response == EOF) {
        if (feof(file))
            syslog(LOG_WARNING, "EOF while waiting for mode response");
        else
            syslog(LOG_ERR, "Read error while waiting for mode response: %s",
                   strerror(errno));
        return -1;
    }
    
    if (response != mode_response) {
        syslog(LOG_INFO, "Invalid mode response from AHRS: %#x", response);
        return -1;
    }
    
    return 0;
}


/**
 * Search for an AHRS header in the stream.
 * @param AHRS400 serial port stream.
 * @return 0 if header found, -1 if error or EOF.
 */
static int ahrs_search_header(FILE *file) {
    for (;;) {
        //Get the next character from the stream
        int recv = fgetc(file);
        
        if (recv == AHRS_DATA_HEADER)
            return 0;
        
        if (recv == EOF) {
            if (feof(file))
                syslog(LOG_WARNING, "EOF while waiting for header");
            else
                syslog(LOG_ERR, "Read error while waiting for header: %s",
                       strerror(errno));
            return -1;
        }
    }
}


/**
 * Calculate message checksum.
 * @param message payload.
 * @param message payload length (without header or checksum).
 * @return message checksum.
 */
static uint8_t checksum(uint8_t *payload, unsigned size) {
    uint8_t checksum = 0;
    for (int i=0; i<size; i++)
        checksum += payload[i];
    return checksum;
}


/**
 * Get current time in microseconds since epoch.
 */
static inline uint64_t get_time_us() {
    struct timespec t;
    if (clock_gettime(CLOCK_REALTIME, &t)) {
        syslog(LOG_ERR, "Error getting time: %s", strerror(errno));
        return 0;
    }

    return t.tv_sec * 1000 + t.tv_nsec / 1000;
}


/**
 * Get a message from the AHRS.
 * @param AHRS400 serial port stream.
 * @param packet payload size (without header or checksum).
 * @param[out] pointer to where the payload should be stored.
 * @param[out] reception time in microseconds since epoch.
 * @return 0 if message read and payload stored, -1 if error or EOF.
 */
int ahrs_get_msg(FILE *file, unsigned size, uint8_t *payload,
                 uint64_t *recv_timestamp) {
    uint8_t work[size + 1];
    uint8_t work_ptr = 0;
    bool header_found = false;
    
    for (;;) {
        // Look for header
        if (!header_found) {
            if (ahrs_search_header(file))
                return -1;
        }

        // Save the time the header was found
        if (recv_timestamp)
            *recv_timestamp = get_time_us();
        
        // Get message body and checksum
        if (!fread(work + work_ptr, sizeof(work) - work_ptr, 1, file)) {
            if (feof(file))
                syslog(LOG_WARNING, "EOF while waiting for payload");
            else
                syslog(LOG_ERR, "Read error while waiting for payload: %s",
                       strerror(errno));
            return -1;
        }
        
        // Check checksum
        uint8_t recv_checksum = work[size];
        if (checksum(work, size) == recv_checksum) {
            memcpy(payload, work, size);
            return 0;
        }
        
        // Look for header in work buffer
        work_ptr = 0;
        header_found = false;
        for (int i=0; i<sizeof work; i++) {
            if (work[i] == AHRS_DATA_HEADER) {
                memmove(work, work + i + 1, sizeof(work) - i - 1);
                work_ptr = sizeof(work) - i - 1;
                header_found = true;
                break;
            }
        }
    }
}


static inline int16_t pack_int16(uint8_t *payload, unsigned index) {
    unsigned msb = index*2;
    unsigned lsb = index*2 + 1;
    return payload[lsb] + ((int16_t)payload[msb]<<8);
}


static inline double raw_to_angle(int16_t raw){
    return raw * M_PI / 32768.0;
}


static inline double raw_to_gyro(int16_t raw){
    return raw * 1.5 * AHRS_GYRO_RANGE / 32768.0;
}


static inline double raw_to_accel(int16_t raw){
    return raw * 1.5 * AHRS_G_RANGE * 9.8 / 32768.0;
}


static inline double raw_to_mag(int16_t raw){
    return raw * 1.5 * 1.25e-4 / 32768.0;
}


static inline double raw_to_temperature(int16_t raw){
    return ((raw * 5 / 4096.0) - 1.375) * 44.44;
}


static inline double raw_to_time(int16_t raw){
    return -raw * 0.00000079;
}


void ahrs_parse_angle(uint8_t *payload, fdas3_ahrs400_angle_t *data) {
    data->angle[0] = raw_to_angle(pack_int16(payload, 0));
    data->angle[1] = raw_to_angle(pack_int16(payload, 1));
    data->angle[2] = raw_to_angle(pack_int16(payload, 2));
    data->gyro[0] = raw_to_gyro(pack_int16(payload, 3));
    data->gyro[1] = raw_to_gyro(pack_int16(payload, 4));
    data->gyro[2] = raw_to_gyro(pack_int16(payload, 5));
    data->accel[0] = raw_to_accel(pack_int16(payload, 6));
    data->accel[1] = raw_to_accel(pack_int16(payload, 7));
    data->accel[2] = raw_to_accel(pack_int16(payload, 8));
    data->mag[0] = raw_to_mag(pack_int16(payload, 9));
    data->mag[1] = raw_to_mag(pack_int16(payload, 10));
    data->mag[2] = raw_to_mag(pack_int16(payload, 11));
    data->temperature = raw_to_temperature(pack_int16(payload, 12));
    data->sensor_time = raw_to_time(pack_int16(payload, 13));
}

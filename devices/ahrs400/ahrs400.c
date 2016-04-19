/**
 * Device module for Crossbow's AHRS400 Attitude and Heading Reference System.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>


/*** AHRS constants ***/
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
    int fd = open(file, O_RDWR);
    if (fd < 0) {
        syslog(LOG_ERR, "Error opening AHRS file: %s", strerror(errno));
        return NULL;
    }
    
    FILE *file = fdopen(fd, "rw");
    if (file == NULL) {
        syslog(LOG_ERR, "Error in fdopen: %s", strerror(errno));
        fclose(fd);
        return NULL;
    }
    
    struct termios ahrs_termios;
    if (tcgetattr(fd, &ahrs_termios)
        || cfsetispeed(&ahrs_termios, AHRS_DEFAULT_BAUDRATE)
        || cfsetospeed(&ahrs_termios, AHRS_DEFAULT_BAUDRATE)
        || tcsetattr(fd, &ahrs_termios)) {
        syslog(LOG_WARNING, "Error setting AHRS baud rate: %s",strerror(errno));
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
        syslog(LOG_ERROR, "ERROR writing ping to AHRS stream: %s",
               strerror(errno));
        return -1;
    }

    int response = fgetc(file);
    if (response == EOF) {
        if (feof(file))
            syslog(LOG_WARNING, "EOF while waiting for ping response");
        else
            syslog(LOG_ERROR, "Read error while waiting for ping response: %s",
                   strerror(file));
        return -1;
    }
    
    if (response != PING_RESPONSE) {
        syslog(LOG_INFO, "Invalid ping from AHRS: %#x", response);
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
        
        if (recv == EOF) {
            if (feof(file))
                syslog(LOG_WARNING, "EOF while waiting for header");
            else
                syslog(LOG_ERROR, "Read error while waiting for header: %s",
                       strerror(file));
            return -1;
        }

        if (recv == AHRS_DATA_HEADER)
            return 0;
    }
}


/**
 * Get a message from the AHRS.
 * @param AHRS400 serial port stream.
 * @param packet payload size.
 * @param[out] pointer to where the payload should be stored.
 * @return 0 if message read and payload stored, -1 if error or EOF.
 */
int ahrs_get_msg(FILE *file, unsigned size, uint8_t *payload) {
    uint8_t work[size + 1];
    uint8_t work_ptr = 0;
    bool header_found = false;
    
    for (;;) {
        // Look for header
        if (header_found) {
            if (ahrs_search_header(file))
                return -1;
        }

        // Get message body and checksum
        if (!fread(work + work_ptr, sizeof(work) - work_ptr, 1, file)) {
            if (feof(file))
                syslog(LOG_WARNING, "EOF while waiting for payload");
            else
                syslog(LOG_ERROR, "Read error while waiting for payload: %s",
                       strerror(file));
            return -1;
        }
        
        // Calculate checksum
        uint8_t calculated_chk = 0;
        for (int i=0; i<size; i++)
            calculated_chk += work[i];

        // Check message
        if (calculated_chk == work[size]) {
            memcpy(payload, work, size);
            return 0;
        }

        // Look for header in work buffer
        work_ptr = 0;
        header_found = false;
        for (int i=0; i<sizeof work; i++) {
            if (work[i] == AHRS_DATA_HEADER) {
                memcpy(work, work + i + 1, sizeof(work) - i - 1);
                work_ptr = sizeof(work) - i - 1;
                header_found = true;
                break;
            }
        }
    }
}

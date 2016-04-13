/**
 * Device module for Crossbow's AHRS400 Attitude and Heading Reference System.
 */


#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>


/*** AHRS constants ***/
#define AHRS_DEFAULT_BAUDRATE B38400

#define AHRS_MSG_HEADER 0xFF
#define AHRS_MSG_LEN 29 //header not included (crc included)


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


int open_ahrs(char *file) {
    int fd = open(file, O_RDWR);
    if (fd < 0) {
        perror("Error opening AHRS file");
        return -1;
    }
    
    struct termios ahrs_termios;
    if (tcgetattr(fd, &ahrs_termios)) {
        perror("Error getting AHRS serial port configuration");
        return -1;
    }
    if (cfsetispeed(&ahrs_termios, AHRS_DEFAULT_BAUDRATE)) {
        perror("Error setting  AHRS baud rate");
        return -1;
    }
    if (tcsetattr(fd, &ahrs_termios)) {
        perror("Error setting AHRS serial port configuration");
        return -1;
    }
    return fd;
}


int config_ahrs(int fd){
    // Put system into polled mode for configuration
    uint8_t data = POLLED_MODE;
    write(fd, &data, sizeof data);
    fsync(fd);
    
    // Discard all unread data in the buffer
    tcflush(fd, TCIFLUSH);

    // Ping the device
    data = PING;
    write(fd, &data, sizeof data);
    read(fd, &data, sizeof data);
    if (data != PING_RESPONSE) {
        fprintf(stderr, "Invalid response to ping: %c.\n", data);
        return -1;
    }
    
    // Get AHRS version
    // TODO

    // Put AHRS in angle mode
    data = ANGLE_MODE;
    write(fd, &data, sizeof data);
    read(fd, &data, sizeof data);
    if (data != ANGLE_MODE_RESPONSE) {
        fprintf(stderr, "Invalid response to angle mode: %c.\n", data);
        return -1;
    }

    // Put AHRS in continuous mode
    data = CONTINUOUS_MODE;
    write(fd, &data, sizeof data);
    read(fd, &data, sizeof data);
    if (data != CONTINUOUS_MODE_RESPONSE) {
        fprintf(stderr, "Invalid response to continuous mode: %c.\n", data);
        return -1;
    }
}

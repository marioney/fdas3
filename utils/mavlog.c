

/**
 * Device module for Crossbow's AHRS400 Attitude and Heading Reference System.
 */


#include <argp.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <termios.h>
#include <unistd.h>

#include "mavlink/v1.0/ceaufmg/mavlink.h"
#include "./utils.h"


/** Program version. */
const char *argp_program_version = "mavlog 0.1";

/** Bug report address. */
const char *argp_program_bug_address = "https://github.com/cea-ufmg/fdas3";

/** Program documentation. */
static char doc[] = "mavlog -- Log a mavlink serial stream.";

/** Description of the accepted arguments. */
static char args_doc[] = "DEVICE LOGFILE";

/** Program options structure. */
static struct argp_option options[] = {
    {0}
};

/** Program arguments structure. */
typedef struct arguments {
    char *device;
    char *logfile;
} arguments_t;


/** Argument parser function */
static error_t parse_opt (int key, char *arg, struct argp_state *state) {
    //Get the arguments structure to write the parsed options
    arguments_t *arguments = state->input;
    
    switch (key) {
    case ARGP_KEY_ARG:
        if (state->arg_num == 0)
            arguments->device = arg;
        else if (state->arg_num == 1)
            arguments->logfile = arg;
        else
            argp_error(state, "Too many arguments.");
        break;

    case ARGP_KEY_END:
        if (state->arg_num < 2)
            argp_error(state, "Not enough arguments.");
        break;
        
    default:
        return ARGP_ERR_UNKNOWN;        
    }
    
    return 0;
}


/** Argument parser object. */
static struct argp argp = {options, parse_opt, args_doc, doc};


/**
 * Open the serial port.
 * Aborts the program on error.
 */
int open_serial_port(char *device) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        char *msg = "Error opening serial port `%s`: %s";
        syslog(LOG_ERR, msg, device, strerror(errno));
        exit(EXIT_FAILURE);
    }
        
    struct termios termios;
    if (tcgetattr(fd, &termios)
        || cfsetispeed(&termios, B57600)
        || tcsetattr(fd, TCSANOW, &termios)) {
        char *msg = "Error setting serial port baud rate: %s";
        syslog(LOG_WARNING, msg, strerror(errno));
    } else {
        cfmakeraw(&termios);
        if (tcsetattr(fd, TCSANOW, &termios)) {
            char *msg = "Error making serial port raw: %s";
            syslog(LOG_WARNING, msg, strerror(errno));
        }
    }
    
    return fd;
}


/**
 * Open the logfile.
 * Aborts the program on error.
 */
FILE* open_log(char *filename) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        char *msg = "Error opening log file %s: %s";
        syslog(LOG_ERR, msg, filename,  strerror(errno));
        exit(EXIT_FAILURE);
    }
}


void logwrite(FILE *log, mavlink_message_t *msg) {
    uint64_t timestamp_be = htobe64(get_time_us());
    if (!fwrite(&timestamp_be, sizeof timestamp_be, 1, log))
        syslog(LOG_ERR, "Error writing timestamp to log: %s", strerror(errno));

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t len = mavlink_msg_to_send_buffer(buf, msg);
    if (!fwrite(buf, len, 1, log))
        syslog(LOG_ERR, "Error writing message to log: %s", strerror(errno));
}


int main(int argc, char **argv) {
    // Parse command line arguments
    arguments_t arguments = {};
    argp_parse(&argp, argc, argv, 0, 0, &arguments);
    
    // Setup syslog
    openlog(0, LOG_PERROR, 0);
    
    // Open the output streams
    int port = open_serial_port(arguments.device);
    FILE *log = open_log(arguments.logfile);    

    // Read loop
    mavlink_message_t msg;
    mavlink_status_t status;
    
    for (;;) {
        char c;
        int n = read(port, &c, 1);
        if (n == 1) {
            if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status))
                logwrite(log, &msg);
        } else if (n < 0) {
            syslog(LOG_ERR, "Error reading serial port: %s", strerror(errno));
        }
    }
    
    return EXIT_SUCCESS;
}

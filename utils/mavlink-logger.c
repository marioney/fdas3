/**
 * Device module for Crossbow's AHRS400 Attitude and Heading Reference System.
 */


#include <argp.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>


#include "mavlink/v1.0/ceaufmg/mavlink.h"


/** Program version. */
const char *argp_program_version = "mavlink-logger 0.1";

/** Bug report address. */
const char *argp_program_bug_address = "https://github.com/cea-ufmg/fdas3";

/** Program documentation. */
static char doc[] = "mavlink-logger -- Log MAVLink messages.";

/** Description of the accepted arguments. */
static char args_doc[] = "SERIAL_PORT";

/** Program options structure. */
static struct argp_option options[] = {
    {"logtxt", 't', "FILE", 0, "Write received data as text to FILE"},    
    {0}
};

/** Program arguments structure. */
typedef struct arguments {
    char *port;
    char *text_log;
} arguments_t;

/** Argument parser function */
static error_t parse_opt (int key, char *arg, struct argp_state *state) {
    //Get the arguments structure to write the parsed options
    arguments_t *arguments = state->input;
    
    switch (key) {
    case 't':
        arguments->text_log = arg;
        break;
        
    case ARGP_KEY_ARG:
        if (state->arg_num >= 1)
            argp_error(state, "Too many arguments.");
        arguments->port = arg;
        break;
        
    case ARGP_KEY_END:
        if (state->arg_num < 1)
            argp_error(state, "Not enough arguments.");
        break;
        
    default:
        return ARGP_ERR_UNKNOWN;
    }
    
    return 0;
}


/** Argument parser object. */
static struct argp argp = {options, parse_opt, args_doc, doc};


FILE *open_text_log(char *path) {
    // Open log file
    if (!path)
        return NULL;
    
    FILE *log = fopen(path, "w");
    if (!log) {
        syslog(LOG_ERR, "Error opening text log: %s", strerror(errno));
        exit(EXIT_FAILURE);
    } 
    return log;
}


int open_serial_port(char *path) {
    // Open the port
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        syslog(LOG_ERR, "Error opening port `%s`: %s", path, strerror(errno));
        exit(EXIT_FAILURE);
    }
    
    // Configure the baud rate
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


void log_message(mavlink_message_t *msg, FILE *text_log) {
    if (!text_log)
        return;
    
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_DATA_INT:
        {
            mavlink_data_int_t payload;
            mavlink_msg_data_int_decode(msg, &payload);
            fprintf(text_log, "%llu\t%hu\t%lld\t",
                    (unsigned long long)payload.time_usec,
                    (unsigned short)payload.id,
                    (long long)payload.value);
        }
        break;

    case MAVLINK_MSG_ID_DATA_FLOAT:
        {
            mavlink_data_float_t payload;
            mavlink_msg_data_float_decode(msg, &payload);
            fprintf(text_log, "%llu\t%hu\t%e\t",
                    (unsigned long long)payload.time_usec,
                    (unsigned short)payload.id, payload.value);
        }
        break;

    case MAVLINK_MSG_ID_DATA_DOUBLE:
        {
            mavlink_data_double_t payload;
            mavlink_msg_data_double_decode(msg, &payload);
            fprintf(text_log, "%llu\t%hu\t%e\t",
                    (unsigned long long)payload.time_usec,
                    (unsigned short)payload.id, payload.value);
        }
        break;
        
    default:
        return;
    }

    printf("msgid %d\n", msg->msgid);
    fprintf(text_log, "%d\t%d\t\%d\n", msg->sysid, msg->compid, msg->msgid);
    fflush(text_log);
}


int main(int argc, char **argv) {
    // Parse command line arguments
    arguments_t arguments = {};
    argp_parse(&argp, argc, argv, 0, 0, &arguments);

    // Open text log
    FILE *text_log = open_text_log(arguments.text_log);

    // Open the serial port
    int port = open_serial_port(arguments.port);

    for (;;) {
        char c;
        int n  = read(port, &c, 1);
        
        if (n < 0) {
            syslog(LOG_ERR, "Error in read: %s", strerror(errno));
            exit(EXIT_FAILURE);
        } else if (n == 0)
            continue;
        
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            log_message(&msg, text_log);
        }
    }
}

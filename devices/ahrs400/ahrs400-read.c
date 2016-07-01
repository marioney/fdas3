/**
 * Device module for Crossbow's AHRS400 Attitude and Heading Reference System.
 */


#include <argp.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <unistd.h>

#include "ahrs400.h"


/** Program version. */
const char *argp_program_version = "ahrs400-read 0.1";

/** Bug report address. */
const char *argp_program_bug_address = "https://github.com/cea-ufmg/fdas3";

/** Program documentation. */
static char doc[] = "ahrs400-read -- Read from a Crossbow AHRS400.";

/** Description of the accepted arguments. */
static char args_doc[] = "PORT";

/** Program options structure. */
static struct argp_option options[] = {
    {"logtxt", 't', "FILE", 0, "Write received data as text to FILE"},
    {"logbin", 'b', "FILE", 0, "Write binary MAVLink stream FILE"},
    {"udp", 'u', 0,  0, "Send MAVLink messages as UDP"},
    {"udp-host", 'h', "HOST", 0, "Host to send MAVLink messages via UDP,"
     " defaults to 224.0.0.1, implies --udp"},
    {"udp-port", 'p', "PORT", 0, "Port to send MAVLink messages via UDP,"
     " defaults to 57600, implies --udp"},
    {0}
};

/** Program arguments structure. */
struct arguments {
    char *port;
    char *text_log;
    char *binary_log;
    bool use_udp;
    char *udp_host;
    short udp_port;
};

/** Program output streams structure */
struct output_streams {
    int udp_sock;
    FILE *binary_log;
    FILE *text_log;
};

/** Argument parser function */
static error_t parse_opt (int key, char *arg, struct argp_state *state) {
    //Get the arguments structure to write the parsed options
    struct arguments *arguments = state->input;
    
    switch (key) {
    case 't':
        arguments->text_log = arg;
        break;

    case 'b':
        arguments->binary_log = arg;
        break;

    case 'h':
        arguments->udp_host = arg;
        break;

    case 'p':
	arguments->use_udp = true;
	{
	    char *endptr = 0;
	    unsigned long port = strtoul(arg, &endptr, 0);
	    if (*endptr) {
		return EINVAL; // Error converting argument to integer
	    }
	    arguments->udp_port = port;
	}
        break;
	
    case 'u':
        arguments->use_udp = true;
        break;
        
    case ARGP_KEY_ARG:
      if (state->arg_num >= 1)
          argp_usage(state);//Too many arguments
      arguments->port = arg;
      break;

    case ARGP_KEY_END:
        if (state->arg_num < 1)
            argp_usage(state);//Not enough arguments
      break;
        
    default:
        return ARGP_ERR_UNKNOWN;        
    }
    
    return 0;
}

/** Argument parser object. */
static struct argp argp = {options, parse_opt, args_doc, doc};


/**
 * Open the program output streams
 */
void open_output_streams(struct arguments *args, struct output_streams *out) {
    // Open text log
    if (args->text_log) {
	out->text_log = fopen(args->text_log, "w");
	if (!out->text_log) {
	    syslog(LOG_ERR, "Error opening text log: %s", strerror(errno));
	    exit(EXIT_FAILURE);
	}
    }

    // Open binary log
    if (args->binary_log) {
	out->binary_log = fopen(args->binary_log, "w");
	if (!out->binary_log) {
	    syslog(LOG_ERR, "Error opening binary log: %s", strerror(errno));
	    exit(EXIT_FAILURE);
	}
    }
    
    // Open UDP socket
    if (args->use_udp) {
	out->udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (out->udp_sock < 0) {
	    syslog(LOG_ERR, "Error creating UDP socket: %s", strerror(errno));
	    exit(EXIT_FAILURE);
	}
    }
}

int main(int argc, char **argv) {
    // Parse command line arguments
    struct arguments arguments = {0};
    struct output_streams output_streams = {-1};
    argp_parse(&argp, argc, argv, 0, 0, &arguments);

    // Setup syslog
    openlog(0, LOG_PERROR, 0);

    // Open the output streams
    open_output_streams(&arguments, &output_streams);
    
    // Open AHRS port
    FILE *stream = ahrs_open(arguments.port);
    if (!stream)
        return EXIT_FAILURE;

    // Put AHRS into polled mode for configuration
    if (ahrs_set_polled(stream))
        return EXIT_FAILURE;
        
    // Wait for pending data to arrive and clear buffers
    fflush(stream);
    sleep(1);
    ahrs_purge(stream);
    
    // Ping the AHRS
    if (ahrs_ping(stream))
        return EXIT_FAILURE;
    
    // Set the mode
    if (ahrs_set_mode(stream, AHRS_ANGLE_MODE)
        || ahrs_set_continuous(stream))
        return EXIT_FAILURE;

    // Read loop
    for (;;) {
        mavlink_ahrs400_angle_raw_t angle_raw;
        mavlink_ahrs400_angle_t angle;
        
        if (ahrs_get_angle_raw(stream, &angle_raw))
            return EXIT_FAILURE;

        ahrs_angle_conv(&angle_raw, &angle);
        
        printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
               angle.xacc, angle.yacc, angle.zacc,
               angle.xgyro, angle.ygyro, angle.zgyro,
               angle.roll, angle.pitch, angle.yaw);
    }
    
    return 0;
}

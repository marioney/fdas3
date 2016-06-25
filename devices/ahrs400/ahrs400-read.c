/**
 * Device module for Crossbow's AHRS400 Attitude and Heading Reference System.
 */


#include <argp.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
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
  {"logbin", 'b', "FILE", 0, "Write binary Mavlink stream FILE"},
  {"udp", 'u', "ADDR:PORT", 0, "Send Mavlink messages as UDP to ADDR:PORT"},
  {0}
};

/** Program arguments structure. */
struct arguments {
    char *port;
    char *text_log;
    char *binary_log;
    char *udp_dest;
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

    case 'u':
        arguments->udp_dest = arg;
        break;
        
    case ARGP_KEY_ARG:
      if (state->arg_num >= 1)
          argp_usage(state);//Too many arguments
      arguments->port = arg;
      break;

    case ARGP_KEY_END:
        if (state->arg_num < 1)
            argp_usage (state);//Not enough arguments
      break;
        
    default:
        return ARGP_ERR_UNKNOWN;        
    }
    
    return 0;
}


/** Argument parser object. */
static struct argp argp = {options, parse_opt, args_doc, doc};


int main(int argc, char **argv) {
    // Parse command line arguments
    struct arguments arguments = {0};
    argp_parse(&argp, argc, argv, 0, 0, &arguments);

    // Setup syslog
    openlog(0, LOG_PERROR, 0);

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

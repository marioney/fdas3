/**
 * Device control for Crossbow's AHRS400 Attitude and Heading Reference System.
 */

#ifndef AHRS400_H
#define AHRS400_H

#include <stdint.h>
#include <stdio.h>

#include "generated/ahrs400_messages/mavlink.h"

#define AHRS_ANGLE_PAYLOAD_LEN 28

typedef enum {
    AHRS_VOLTAGE_MODE,
    AHRS_SCALED_MODE,
    AHRS_ANGLE_MODE
} ahrs_mode_t;

FILE* ahrs_open(char *path);
int ahrs_ping(FILE *file);
int ahrs_set_continuous(FILE *file);
int ahrs_set_polled(FILE *file);
int ahrs_purge(FILE *file);
int ahrs_set_mode(FILE *file, ahrs_mode_t mode);
int ahrs_get_angle_raw(FILE *file, mavlink_ahrs400_angle_raw_t *angle_raw);
void ahrs_angle_conv(mavlink_ahrs400_angle_raw_t *raw,
                     mavlink_ahrs400_angle_t *scaled);


#endif//AHRS400_H

/**
 * Device control for Crossbow's AHRS400 Attitude and Heading Reference System.
 */

#ifndef AHRS400_H
#define AHRS400_H

#include <stdint.h>
#include <stdio.h>

#include "fdas3_ahrs400_angle_t.h"

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
int ahrs_get_msg(FILE *file, unsigned size, uint8_t *payload,
                 uint64_t *recv_timestamp);
void ahrs_parse_angle(uint8_t *payload, fdas3_ahrs400_angle_t *data);


#endif//AHRS400_H

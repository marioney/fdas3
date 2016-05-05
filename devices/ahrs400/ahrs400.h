/**
 * Device control for Crossbow's AHRS400 Attitude and Heading Reference System.
 */

#ifndef AHRS400_H
#define AHRS400_H

#include <stdint.h>
#include <stdio.h>

#include "fdas3_ahrs400_angle_t.h"

#define AHRS_ANGLE_PAYLOAD_LEN 28


FILE* ahrs_open(char *path);
int ahrs_ping(FILE *file);
int ahrs_get_msg(FILE *file, unsigned size, uint8_t *payload,
                 uint64_t *recv_timestamp);
void ahrs_parse_angle(uint8_t *payload, fdas3_ahrs400_angle_t *data);


#endif//AHRS400_H

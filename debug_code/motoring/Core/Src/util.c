/*
 * util.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 *
 *      Assorted utilities used by the project
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "util.h"

/*
 * Convert array to hexadecimal string
 *
 * ar: pointer to the array
 * len: length of the array
 * buf: char buffer where the string is stored
 */
void array8_to_hex(uint8_t *ar, size_t len, char *buf)
{
    char temp[3];
    int j = 0;
    for (int i = 0; i < len; i++) {
        sprintf(temp, "%02X", ar[i]);
        memcpy(&buf[j], temp, 2);
        j += 2;
        if (i < len-1) {
            buf[j] = ' ';
            j++;
        }
    }
    buf[j] = '\0';
}

/*
 * Dynamixel Protocl checksum
 *
 * buffer: pointer to the array with the dynamixel commands
 *
 * returns: checksum according to https://emanual.robotis.com/docs/en/dxl/protocol1/#checksum-instruction-packet
 */
uint8_t dmp_chksm(uint8_t* buffer)
{
	uint32_t checksum = 0;
	// Length is the fourth field
	for (int i = 0; i < buffer[3] + 1; i++) {
		checksum += buffer[2 + i];
	}
	return ~((uint8_t)checksum);
}

/* Min value of array
 * array: the array
 * len: length of the array
 *
 * returns: minimum value of the array
 */
int min_of_array(int *array, int len)
 {
	int min = array[0];
	for (int i = 0; i < len; i++) {
		if (array[i] < min) {
			min = array[i];
		}
	}
	return min;
}

/* Max value of array
 * array: the array
 * len: length of the array
 *
 * returns: maximum value of the array
 */
int max_of_array(int *array, int len)
{
	int max = array[0];
	for (int i = 0; i < len; i++) {
		if (array[i] > max) {
			max = array[i];
		}
	}
	return max;
}

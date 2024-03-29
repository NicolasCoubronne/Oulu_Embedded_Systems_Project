/*
 * util.h
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 *
 *      Assorted utilities used by the project (header)
 *      More documentation in implementation file
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_


void array8_to_hex(uint8_t *ar, size_t len, char *buf);
uint8_t dmp_chksm(uint8_t* buffer);
int min_of_array(int *array, int len);
int max_of_array(int *array, int len);

#endif /* INC_UTIL_H_ */

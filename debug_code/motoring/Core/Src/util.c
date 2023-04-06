/*
 * util.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 */

#include <stdio.h>

/*
 * Print array in hexadecimal (probably uart buffer for ax12)
 */
void aprint(uint8_t *ar, int len)
{
	for (int i=0; i<len; i++) {
		printf("%02x ", ar[i]);
	}
}

/*
 * print.h
 *
 *  Created on: Aug 14, 2022
 *      Author: petarlalov
 */

#ifndef INC_PRINT_H_
#define INC_PRINT_H_

void print_init(int (*print_service)(const char *str));

int println(const char *format, ...);
int print(const char *format, ...);


#endif /* INC_PRINT_H_ */

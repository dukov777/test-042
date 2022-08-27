#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "print.h"

#define PRINT_BUFFER_SIZE 256
static char printBuffer[PRINT_BUFFER_SIZE];


static int (*__print_service)(const char *str);

void print_init(int (*print_service)(const char *str)){
	__print_service = print_service;
}

int _print(const char *str) {
    return __print_service(str);
}

int _printf(const char *format, va_list args) {

    if(strlen(format) >= PRINT_BUFFER_SIZE) {
		return -1;
	}

	vsnprintf(printBuffer, PRINT_BUFFER_SIZE, format, args);

	return _print(printBuffer);
}

int println(const char *format, ...) {
	va_list args;

	va_start(args, format);
	int error = _printf(format, args);

	va_end(args);

	if (error >= 0){
	    error = _print("\r\n");
	}

	return error;
}

int print(const char *format, ...) {
	va_list args;
	va_start(args, format);
	int error = _printf(format, args);
	va_end(args);
	return error;
}

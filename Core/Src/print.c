#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx_hal.h"
#include "print.h"

#define PRINT_BUFFER_SIZE 256
static char printBuffer[PRINT_BUFFER_SIZE];


#define HAL_ERROR_MSG_MAPPING_LEN 4

const struct {
    uint32_t error_id;
    const char* human_readable_msg;
} __hal_error_msg_mapping[HAL_ERROR_MSG_MAPPING_LEN] = {
        {HAL_OK      , "HAL_OK"},
        {HAL_ERROR   , "HAL_ERROR"},
        {HAL_BUSY    , "HAL_BUSY"},
        {HAL_TIMEOUT , "HAL_TIMEOUT"}
};


const char* hal_error_2_str(HAL_StatusTypeDef error){
    for (int i = 0; i < HAL_ERROR_MSG_MAPPING_LEN; i++) {
        if (error == __hal_error_msg_mapping[i].error_id) {
            return  __hal_error_msg_mapping[i].human_readable_msg;
        }
    }
    return "No such HAL Error!";
}


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

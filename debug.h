#ifndef DEBUG_H
#define DEBUG_H

#include "stm32u5xx_hal.h"

void Debug_Init(UART_HandleTypeDef *uart);
void Debug_Printf(const char *format, ...);

#endif

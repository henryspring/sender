#include "debug.h"

#include <stdarg.h>
#include <stdio.h>

static UART_HandleTypeDef *debug_uart;

void Debug_Init(UART_HandleTypeDef *uart)
{
    debug_uart = uart;
}

void Debug_Printf(const char *format, ...)
{
    if (debug_uart == NULL) {
        return;
    }

    char line[160];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);

    if (length <= 0) {
        return;
    }
    if (length >= (int)sizeof(line)) {
        length = (int)sizeof(line) - 1;
    }

    HAL_UART_Transmit(debug_uart, (uint8_t *)line, (uint16_t)length, 250u);
}

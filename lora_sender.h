#ifndef LORA_SENDER_H
#define LORA_SENDER_H

#include "stm32u5xx_hal.h"
#include <stdint.h>

void LoRaSender_Init(UART_HandleTypeDef *uart);
void LoRaSender_Poll(void);
int LoRaSender_IsPaired(void);

/* Blocks until complete or failed. Does not copy or take ownership of jpeg. */
int LoRaSender_SendImage(const uint8_t *jpeg, uint32_t size);

#endif

#ifndef MAILBOX_CONFIG_H
#define MAILBOX_CONFIG_H

#include "stm32u5xx_hal.h"

/* Confirm these two pins are unused by the camera and UARTs in CubeMX. */
#define DOOR_PORT               GPIOB
#define DOOR_PIN                GPIO_PIN_0
#define LED_PORT                GPIOB
#define LED_PIN                 GPIO_PIN_1

/* Door switch connects the input to GND when the mailbox is closed. */
#define DOOR_OPEN_LEVEL         GPIO_PIN_SET
#define LED_ON_LEVEL            GPIO_PIN_SET

#define DOOR_DEBOUNCE_MS        50u
#define CLOSED_SETTLE_MS        2500u
#define LED_WARMUP_MS           500u
#define MAILBOX_COOLDOWN_MS     60000u
#define LINK_WAIT_MS            60000u
#define TRANSFER_ATTEMPTS       2u

#define CAMERA_BUFFER_BYTES     60000u
#define CAMERA_MAX_JPEG_BYTES   55000u
#define CAMERA_CAPTURE_TIMEOUT_MS 5000u

/* E32-900T20D settings must match the gateway. */
#define E32_CONFIG_ON_BOOT      1u
#define E32_TARGET_SPED         0x1Cu
#define E32_TARGET_CHANNEL      60u
#define E32_TARGET_OPTION       0x44u

#define LORA_AUX_PORT           GPIOC
#define LORA_AUX_PIN            GPIO_PIN_0
#define LORA_M0_PORT            GPIOC
#define LORA_M0_PIN             GPIO_PIN_1
#define LORA_M1_PORT            GPIOC
#define LORA_M1_PIN             GPIO_PIN_2

#endif

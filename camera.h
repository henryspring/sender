#ifndef CAMERA_H
#define CAMERA_H

#include "stm32u5xx_hal.h"
#include <stdint.h>

int Camera_Init(I2C_HandleTypeDef *i2c, DCMI_HandleTypeDef *dcmi);

/* jpeg points into the camera's static capture buffer until the next capture. */
int Camera_CaptureJPEG(const uint8_t **jpeg, uint32_t *size);

#endif

#ifndef __SPI_H__
#define __SPI_H__

#include "stm32g0xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void);
void BMI088_accel_get_chip_id(uint8_t *accel_chip_id);


#endif /* __SPI_H__ */
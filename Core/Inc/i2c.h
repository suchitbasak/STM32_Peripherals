#ifndef __I2C_H__
#define __I2C_H__

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);
void I2C_scan_devices(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void AHT20_soft_reset(void);
void AHT20_Init(void);
void AHT20_trigger_measurement(void);
void AHT20_get_measurement(uint8_t*);
void AHT20_process_data(uint8_t raw_data[6], float *temperature_c, float *humidity_percent);

#endif
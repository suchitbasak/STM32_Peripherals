#ifndef __SPI_H__
#define __SPI_H__

#include "main.h"

extern SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void);


//helper function
HAL_StatusTypeDef SPI_write_to_register(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t register, uint8_t data);
uint8_t SPI_read_from_register(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg);

HAL_StatusTypeDef BMI088_accel_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);
HAL_StatusTypeDef BMI088_Accel_Init_Final(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);
uint8_t BMI088_accel_chip_id(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);
uint8_t BMI088_accel_dataready(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);

HAL_StatusTypeDef BMI088_accel_sensor_data(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, float *accel_x, float *accel_y, float *accel_z, uint8_t *range, char *buffer);

#endif /* __SPI_H__ */
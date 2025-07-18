//#include "stm32g0xx_hal_spi.h"
#include "spi.h"
#include <stdio.h>

// --- BMI088 Accelerometer Register Definitions
#define BMI088_ACC_CHIP_ID_REG      0x00
#define BMI088_ACC_CONF_REG         0x40
#define BMI088_ACC_PWR_CTRL_REG     0x7D
#define BMI088_ACC_SOFTRESET_REG    0x7E
#define BMI088_ACC_RANGE            0x41
#define ACC_PWR_CONF_REG            0x7C


#define BMI088_ACC_CHIP_ID          0x1E
#define BMI088_ACC_STATUS           0x03

#define G_VALUE                     9.80665f



SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1){
  
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

  }
}



// using IMU BMI088 from Bosch

HAL_StatusTypeDef SPI_write_to_register(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg, uint8_t data){
  HAL_StatusTypeDef status;
  uint8_t tx_buffer[2];
  tx_buffer[0] = reg;
  tx_buffer[1] = data;

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
  
  if (HAL_SPI_Transmit(hspi, tx_buffer, 2, HAL_MAX_DELAY) != HAL_OK){
    status = HAL_ERROR;
  } else{
    status = HAL_OK;
  }

  //signals low at the end of transmission
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  return status;
}


uint8_t SPI_read_from_register(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg){
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2];
  uint8_t data; // data read from the register

  tx_buffer[0] = reg | 0x80; // set MSB of the register address to 1, to indicated a read operation
  tx_buffer[1] = 0x00; // dummy byte

  // pull chip select low to select the correct peripheral
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, HAL_MAX_DELAY);

  data = rx_buffer[1];

  // pull chip select high to free the peripheral
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  return data;

}

void BMI088_accel_soft_reset(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin) {
  HAL_StatusTypeDef status;

  // wait >2ms after powering up the sensor
  HAL_Delay(5);

  // Soft reset the sensor to ensure internal FSMs are initialized
  SPI_write_to_register(hspi, cs_port, cs_pin, BMI088_ACC_SOFTRESET_REG, 0xB6);
  HAL_Delay(50); // Wait for stabilization after soft reset
  return;
}


// now lets initialise the accelerometer
HAL_StatusTypeDef BMI088_accel_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin){
  
  HAL_StatusTypeDef status;


  // Switch accel to SPI mode (it starts in I2C mode by default) by reading from the CSB1 pin
  // Perform a dummy read
  uint8_t chip_id = SPI_read_from_register(hspi, cs_port, cs_pin, BMI088_ACC_CHIP_ID_REG);

  // Switch the ACC from suspend mode to normal mode

  // Write to ACC_PWR_CTRL_REG, to enter "normal mode"
  SPI_write_to_register(hspi, cs_port, cs_pin, BMI088_ACC_PWR_CTRL_REG, 0x04);
  // wait for 450 microseconds
  HAL_Delay(1);
  
  // configure sensor to stay in normal mode
  SPI_write_to_register(hspi, cs_port, cs_pin, ACC_PWR_CONF_REG, 0x00);
  HAL_Delay(10);


  // Set acc configuration to normal BW and ODR = 100 Hz
  SPI_write_to_register(hspi, cs_port, cs_pin, BMI088_ACC_CONF_REG, 0xA8);
  HAL_Delay(10);



  chip_id = SPI_read_from_register(hspi, cs_port, cs_pin, BMI088_ACC_CHIP_ID_REG);

  // now, verify communication
  if (chip_id == BMI088_ACC_CHIP_ID){
    status = HAL_OK;
  } else{
    status = HAL_ERROR;
  }

  return(status);
}


// read Chip ID register for the accelerometer from register 0x00
uint8_t BMI088_accel_chip_id(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin){
  uint8_t chip_id = SPI_read_from_register(hspi, cs_port, cs_pin, BMI088_ACC_CHIP_ID_REG);
  return chip_id;
}


// check status of data ready bit in ACC_STATE
uint8_t BMI088_accel_dataready(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin){
  uint8_t accel_drdy = SPI_read_from_register(hspi, cs_port, cs_pin, BMI088_ACC_STATUS);
  // we only need to look at bit 7: if bit 7 = 1; data is ready to be read from the data registers 0x12 to 0x17
  //accel_drdy = accel_drdy & 0x80;
  return accel_drdy;
}


// read accelerometer data 
HAL_StatusTypeDef BMI088_accel_sensor_data(SPI_HandleTypeDef *hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, float *accel_x_mss, float *accel_y_mss, float *accel_z_mss, uint8_t *range, char *buffer){
  // you need to transmit and receive 7 bytes,
  uint8_t tx_buffer[7] = {0x12 | 0x80, 0, 0, 0, 0, 0, 0};
  uint8_t rx_buffer[7];
  HAL_StatusTypeDef status; // check if this is how you init status in all functions, uniformity

  // pull cs low to select sensor
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

  // spi transaction
  status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 7, HAL_MAX_DELAY);

  // pull cs high to deselect sensor
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  if (status == HAL_OK){
    uint8_t acc_conf = SPI_read_from_register(hspi, cs_port, cs_pin, 0x40)&0b1111;
    sprintf(buffer, ", ACC_CONF: %u\r\n",acc_conf);

    // First byte received in rx_buffer is dummy, second byte onwards, we get LSB and then MSB for each co-ordinate
    int16_t raw_x = (int16_t)((rx_buffer[2] << 8) | rx_buffer[1]);
    int16_t raw_y = (int16_t)((rx_buffer[4] << 8) | rx_buffer[3]);
    int16_t raw_z = (int16_t)((rx_buffer[6] << 8) | rx_buffer[5]);
    
    uint8_t range_reg_val = SPI_read_from_register(hspi, cs_port, cs_pin, BMI088_ACC_RANGE);

    *range = range_reg_val & 0X03;

    float sensitivity = 0.0f;
    switch (range_reg_val & 0x03)
    {
        case 0x00: sensitivity = 10920.0f; break; // ±3g
        case 0x01: sensitivity = 5460.0f;  break; // ±6g
        case 0x02: sensitivity = 2730.0f;  break; // ±12g
        case 0x03: sensitivity = 1365.0f;  break; // ±24g
        default: return HAL_ERROR;
    }

    // Formula is: (raw_value / sensitivity) * g_constant
    *accel_x_mss = ((float)raw_x / sensitivity) * G_VALUE;
    *accel_y_mss = ((float)raw_y / sensitivity) * G_VALUE;
    *accel_z_mss = -1 * ((float)raw_z / sensitivity) * G_VALUE;
  }

  return status;

}


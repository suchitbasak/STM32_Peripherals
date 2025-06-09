#include <stdio.h>
#include <string.h>
#include "i2c.h"

I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

  }
}


void I2C_scan_devices(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart){
  // find I2C devices on the bus
  char buffer[64];
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 10) == HAL_OK) {
      sprintf(buffer, "Found I2C device at 0x%02X\r\n", addr);
      HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
  }
}


void AHT20_soft_reset(void){
  // Send 0xBA to soft reset the temp sensor
  uint8_t reset_cmd = 0xBA;
  HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, &reset_cmd, 1, HAL_MAX_DELAY);
  HAL_Delay(50);  // Wait for reset to complete
}

void AHT20_Init(void){
  // initialise the AHT20
  uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};
  HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, init_cmd, 3, HAL_MAX_DELAY);
  HAL_Delay(50);
}


void AHT20_trigger_measurement(void){
  uint8_t trig_measurement_cmd[3] = {0xAC, 0x33, 0x00};
  HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, trig_measurement_cmd, 3, HAL_MAX_DELAY);
  HAL_Delay(90);
}

void AHT20_get_measurement(uint8_t *raw_data){
  // sennd this to receive one measuremetn
  HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1, raw_data, 6, HAL_MAX_DELAY);
}

// convert sensor data to celsius and % humidity
void AHT20_process_data(uint8_t raw_data[6], float *temperature_c, float *humidity_percent){
  uint32_t raw_temp = 0;
  uint32_t raw_humidity = 0;

  raw_humidity = ((uint32_t)raw_data[1] << 12) | ((uint32_t)raw_data[2] << 4) | ((uint32_t)(raw_data[3]  & 0xF0)>>4);
  raw_temp = ((uint32_t)(raw_data[3] & 0x0F) << 16) | ((uint32_t)raw_data[4] << 8) | (uint32_t)(raw_data[5]);
  
  // AHT20 sensor measures a temperature range of -50°C to +150°C
  // raw temp value / 2^20 (to normalise it)
  *temperature_c = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;
  
  // Similarly humidity is also normalised first and then scaled to a %
  *humidity_percent = ((float)raw_humidity / 1048576.0f) * 100.0f;
}
// pointers because ʸᴼᵤ ᶜᴬₙᴺₒᵀ ᴳₑᵀ ² ᴿₑᵀᵤᴿₙ ᵥᴬₗᵁₑˢ ᶠᵣᴼₘ ₐ ᶠᵁₙᶜₜᴵₒᴺ ᴵₙ ᶜ
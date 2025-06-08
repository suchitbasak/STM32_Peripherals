
//#include "stm32g0xx_hal_conf.h"
#include "stm32g0xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "leds.h"
#include "i2c.h"
#include "usart.h"


void SystemClock_Config(void);

int main(void){
    HAL_Init();
    SystemClock_Config();
    led_onboard_init();

    MX_I2C1_Init();
    MX_USART2_UART_Init();

    AHT20_soft_reset();
    AHT20_Init();

    float temperature, humidity;
    uint8_t raw_data[6];
    char buffer[100]; //for printing

    // Scan and print devices
    I2C_scan_devices(&hi2c1, &huart2);
    // Optional delay
    // HAL_Delay(5000);
    
    while(1){
        led_onboard_toggle();
        
        // reading data from the AHT20
        AHT20_trigger_measurement();
        AHT20_get_measurement(raw_data);
        // Read raw data from sensor
        AHT20_get_measurement(raw_data);

        // Print raw data bytes for debugging
        //sprintf(buffer, "Raw Data: %02X %02X %02X %02X %02X %02X\r\n",
        //    raw_data[0], raw_data[1], raw_data[2],
        //    raw_data[3], raw_data[4], raw_data[5]);

        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        // check if sensor is busy
        if ((raw_data[0] & 0x80) == 0){
            AHT20_process_data(raw_data, &temperature, &humidity);
            sprintf(buffer, "Temperature: %.2f C, Humidity: %.2f%%\r\n", temperature, humidity);         
        } else {
            sprintf(buffer, "Sensor is busy or output is invalid\r\n");
        }
        
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_Delay(1000);
    }
}

void SystemClock_Config(void){
    {
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

        /** Configure the main internal regulator output voltage */
        HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

        /** Initializes the RCC Oscillators according to the specified parameters
         * in the RCC_OscInitTypeDef structure.*/
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /** Initializes the CPU, AHB and APB buses clocks
         */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                    |RCC_CLOCKTYPE_PCLK1;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add THEIR own implementation to report the HAL error return state */
  //__disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
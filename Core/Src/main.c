
//#include "stm32g0xx_hal_conf.h"
#include <string.h>
#include <stdio.h>
#include "main.h"

#define BMI088_ACC_CHIP_ID_REG      0x00
#define BMI088_ACC_CONF_REG         0x40
#define BMI088_ACC_PWR_CTRL_REG     0x7D
#define BMI088_ACC_SOFTRESET_REG    0x7E
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_PWR_CONF_REG     0x7C // From Linux driver - disables suspend mode


void SystemClock_Config(void);


int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_SPI1_Init();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    led_debug_init();

    HAL_StatusTypeDef init_status;
    HAL_StatusTypeDef data_ready_status;
    HAL_StatusTypeDef self_test_status = HAL_OK;
    // uint8_t chipid;
    float raw_x, raw_y, raw_z;
    uint8_t range; //range reg val
    uint8_t accel_drdy;
    char buffer[500]; //for printing

    BMI088_accel_soft_reset(&hspi1, GPIOA, GPIO_PIN_9, &buffer);
    init_status =  BMI088_accel_init(&hspi1, GPIOA, GPIO_PIN_9, &buffer);
    uint8_t chipid = 0;
    HAL_StatusTypeDef accel_data_status;

    chipid = BMI088_accel_chip_id(&hspi1, GPIOA, GPIO_PIN_9);
    sprintf(buffer, "chipid 0x%02X\r\n", chipid);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    sprintf(buffer, "Input sensor name: A: Temp; B: Humidity; C: Accel.");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Uncomment the line below to perform a self test
    // self_test_status =  BMI088_accel_self_test(&hspi1, GPIOA, GPIO_PIN_9, &buffer);

    while(1){

        __NOP();
        
        if(init_status == HAL_OK && self_test_status == HAL_OK) {
            led_debug_on(); // LED on means init was successful
            
            if(chipid == 0x1E) {
                // chip id is okay, let us look at the data ready register
                
                do{
                accel_drdy = SPI_read_from_register(&hspi1, GPIOA, GPIO_PIN_9, 0x03);
                //HAL_Delay
                } while ((accel_drdy & 0x80) == 0);

                // read data
                accel_data_status = BMI088_accel_sensor_data(&hspi1, GPIOA, GPIO_PIN_9, &raw_x, &raw_y, &raw_z, &buffer);
                if (accel_data_status == HAL_OK) {
                sprintf(buffer, "X: %.4f, Y: %.4f, Z: %.4f m/s^2, R: %u\r\n", raw_x, raw_y, raw_z);
                HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
                
                HAL_Delay(1000);
                }
            }

        } else {
            // Handle the case where initialization failed in the first place
            led_debug_toggle();
            HAL_Delay(100);
        }
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
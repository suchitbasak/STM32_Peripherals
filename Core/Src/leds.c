/*functions for leds - onboard or otherwise */

#include "stm32g0b1xx.h"
#include "main.h"
#include "leds.h"

void led_onboard_init(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_5; // confirmed that the on-board led is connected to GPIO_PIN_5
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void led_onboard_toggle(void){
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}


// I have a blue debug LED on PA3 (CN10 Pin 6)
void led_debug_init(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void led_debug_on(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);
}

void led_debug_toggle(void){
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
}
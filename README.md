# STM32 + Peripherals# STM32 + Peripherals

This repository will contain code to interface different peripherals with STM32G0B1xx. I have started with an AHT20 breakout board, connected to the STM32 Nucloeboard via I2C. Temperature and relative humidity measurements are printed via UART. I will be adding support for more peripherals like an IMU over SPI, audio in/ out over I2S, LEDs over GPIO/PWM, another MCU over UART etc.

I haven't decided if I want to make a PCB with the sensors yet.

The .IOC project file is availble and has been used to generate some of the code. However I have rewritten `main.c`, `leds.c` and `i2c.c` since I prefer cleaner code.

## How to build and flash
1. Make a build directory to save files: `mkdir build && cd build`
2. Tell cmake to generate the build system: `cmake ..`
3. Build the project: `cmake --build .` - don't forget the . at the end of the command
4. Flash the generated binary file on to the STM32: `st-flash write filename.bin 0x8000000`

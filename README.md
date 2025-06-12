# STM32 + Peripherals term

Interfacing a few peripherals with the STM32G0B1xx microcontroller, incliding a temperature sensor, IMU, LEDs and a screen.

- AHT20 temperature sensor is connected via I2C, measuring temperature and relative humidity and displaying results over UART.
- I am currently working on adding the writing the IMU code.
- I have not finalised the screen at this point.
- Audio will be added in the next version
- Eventually I want to design an STM32 PCB with all these sensors.

STM32Cube ICube MXDE is used to generate a ot of the code
The .IOC project file is availble and has been used to generate some of the code. However I have rewritten `main.c`, `leds.c` , `i2c.c` etc. since I prefer cleaner code.
 
## How to build and flash
1. Make a build directory to save files: `mkdir build && cd build`
2. Tell cmake to generate the build system: `cmake ..`
3. Build the project: `cmake --build .` - don't forget the . at the end of the command
4. Flash the generated binary file on to the STM32: `st-flash write filename.bin 0x8000000`

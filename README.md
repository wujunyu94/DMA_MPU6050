# DMA_MPU6050
A whole MPU6050 Lib with stmf1 mcu by hardware dma i2c
This project can be opened by KEIL5
The driver code is created by stm32cubeMX

    I2C1 GPIO Configuration :   
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 

MPU6050 raw is read in TIM4 global interupt, it can be found in stm32f1xx_it.c
The raw is saved in struct acc and gyro ,you can transfer it with usart1 or watch them in debug watch window

The MPU6050 default settings is 1000Hz sampling rate, 2000deg/s and ±16g
You can find the initialization function is in i2c.c

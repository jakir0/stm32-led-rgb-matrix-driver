# STM32 led rgb matrix driver
*Written for **STM32 NUCLEO-F446RE**

*Provides communication with PC APP [STM32_USART_GUI](https://github.com/jakir0/stm32-led-rgb-matrix-driver) via USART

*Controls [**MBI5030**](https://www.neumueller.com/datenblatt/macroblock/MBI5030%20Datenblatt%20-%20Datasheet.pdf) 16-Channel Constat Current LED Driver With 16-bit PWM Control

*MBI5030s are connected in cascade and accesed by SPI-like 3 wire interface 

*Number of MBI5030s can be altered by changing proper defines 
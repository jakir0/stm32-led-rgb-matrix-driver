# STM32 led rgb matrix driver

This project was used for my master's thesis. Main goal of this project was to develop a method of displacing the light source in active character displays used in ITS systems and to develop a prototype RGB LED display model with a remote fiber optic panel. Single pixel of said matix was developed, since to create a display there is only a need for replication of it and minor changes to a source code to deal with displaying graphics. Displacing the light source form its projection place was achvived by using a PMMA optical fibre connected to LEDs by optical coupler. 

STM32 NUCLEO-F446RE provides control over cascade of [**Macroblocks MBI5030**](https://www.neumueller.com/datenblatt/macroblock/MBI5030%20Datenblatt%20-%20Datasheet.pdf) (16-Channel Constat Current LED Driver With 16-bit PWM) via SPI-like interface, which drive LEDs making a single pixel. 

STM32 communicates  with PC via USB to control LEDs. For this purpose PC app with GUI was created that can me found [**HERE**](https://github.com/jakir0/STM32_USART_GUI). 

### Video presentation of an [app](https://github.com/jakir0/STM32_USART_GUI), working with STM32 to control the pixel of display:
https://user-images.githubusercontent.com/83252838/135508797-46dc0c34-2e1c-430d-bb18-5bd77ed13df5.mp4

### Single pixel display module with attached PMMA fiber:
![single_pixel](https://user-images.githubusercontent.com/83252838/135523138-3eee72c6-4b69-4f9a-b7c5-e37fb92cc4e4.png)

### Connection between PC and STM32: 
![Circut_diagram_PC_STM](https://user-images.githubusercontent.com/83252838/135514963-b839be2e-03f1-4192-b15c-44f0bdbc3161.png)

### Circut diagram:
![Circut_diagram](https://user-images.githubusercontent.com/83252838/135514670-f09d79be-253a-4b42-afcc-c99188759305.png)

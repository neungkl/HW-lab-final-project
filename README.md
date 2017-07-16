# STM32F4 Hardware Project

_This is project for 2110363 Hardware Synthesis Laboratory study class in Chulalongkorn University_

## Instruction

Design a software with [STM32F4DISCOVERY](http://www.st.com/en/evaluation-tools/stm32f4discovery.html) board, which require 3 sensors to create the embedded program.

1. LIS302DL – 3-axis accelerometer
2. MP45DT02 – Digital Microphone
3. CS43L22 – audio DAC, speaker driver

Your objectives

1. Learn and summarize datasheet from above 3 sensors.
2. Demonstration of these IC
    1. Print accelerometer value via UART
    2. Print microphone volume value via UART
    3. Play sound from UART input
3. Create a new project with your own idea by combine all IC from instruction 2 together.

## Docs

Main codes (only `main.c` files) and documentation uploaded on `/docs/` directory.

### Final Project (Instruction 3)

I try to create a mini game that test performance of your memory.

The game requires STM32 board to play (UART usb is optional), controlled by rotating the board to interact with game _(3-axis accelerometer)_, it'll sound while your playing _(speaker)_.

After you finish the game or your lose, if you try to restart it, just clap one time _(microphone)_ and the game will automatically restart.

Demonstration provided below.

## Demonstration

2.1. LIS302DL – 3-axis accelerometer

[![LIS302DL – 3-axis accelerometer](http://img.youtube.com/vi/X1yB6p0slpo/0.jpg)](https://youtu.be/X1yB6p0slpo?list=PLFt9kotlOsQLRj9_FBZgsDHt8URf2Fik9)

2.2. MP45DT02 – Digital Microphone

[![MP45DT02 – Digital Microphone](http://img.youtube.com/vi/GVRDzz0FhP0/0.jpg)](https://youtu.be/GVRDzz0FhP0?list=PLFt9kotlOsQLRj9_FBZgsDHt8URf2Fik9)

2.3. CS43L22 – audio DAC, speaker driver

[![CS43L22 – audio DAC, speaker driver](http://img.youtube.com/vi/Aepxf0yS_kM/0.jpg)](https://youtu.be/Aepxf0yS_kM?list=PLFt9kotlOsQLRj9_FBZgsDHt8URf2Fik9)


3. Final Project

[![CS43L22 – audio DAC, speaker driver](http://img.youtube.com/vi/02R9VJlYNHY/0.jpg)](https://youtu.be/02R9VJlYNHY?list=PLFt9kotlOsQLRj9_FBZgsDHt8URf2Fik9)

## License

[MIT License](LICENSE) Copyright (c) 2016 Kosate Limpongsa
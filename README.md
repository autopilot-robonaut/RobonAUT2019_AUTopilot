# RobonAUT2019 AUTopilot
Autonomous model car software.


# SUMMARY

The goal of the competition was to create an autonomous car, which can navigate in a labyrinth, change lanes, follow a safety car and then compete in a speed race. The detailed rule book is also in the project.

The website of the competition: http://robonaut.aut.bme.hu/
Video of the competition (our robot can be seen at 1:01:00): https://bsstudio.hu/video/robonaut-2019-kozvetites


# HARDWARE

The brain of the car was a STM32F446RE developer board.
## Sensors
* 96 TCRT5000 reflective sensors for line following
* BOSCH BNO055 absolute orientation sensor
* Sharp GP2Y0A02YK0F analog distance sensor


# SOFTWARE STRUCTURE

The code is written in C language. The project is based on FreeRTOS and consists of several tasks.
Tasks:
* Multichannel ADC reading (DMA)
* Line following with state space controller
* Motor speed with PID controller
* Labyrinth solving algorithm (Dijkstra's algorithm, Graph-based)
 

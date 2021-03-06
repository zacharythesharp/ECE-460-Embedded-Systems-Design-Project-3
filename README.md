# ECE 460 Embedded Systems Design Project 3

**Introduction**

The overall goals of the system were to create a code base that allowed the touch screen to be
used for various purposes, without completely halting other processes. The base code functions
as such - when the upper part of the LCD screen is used to draw, the HBLED stops flashing.

With the project complete, the LCD can be drawn on while allowing the HBLED to continue to
flash. In addition to this, the brightness of the flash can be altered by adjusting where the screen
is touched on the lower end near the “Dim / Bright” display string.

The base code was supplied by Dr. Alexander Dean from North Carolina State University. The objective
was to modify the code base in order to achieve the desired outcome. The files I primarliy worked in were:
- debug.h
- my_data.h
- main.c
- thread.c
- control.c
- touchscreen.c

This code was run on an NXP MKL25Z MCU. It was developed in the Keil uVision5 IDE using an RTOS2-RTX5 package.
In this repository is a folder called 'Project 3'. The main folders in this project are as follows:
1. Include: contains all the necessary header files
2. Source: contains all the source files
3. All the other files and folders are generated by the Keil IDE. 

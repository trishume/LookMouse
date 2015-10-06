# LookMouse

A one weekend hobby project to try making a mouse that was controlled by head movement read by an IMU attached to a microphone headset.
The IMU is connected to the microcontroller (in a 3D printed custom box) via a removable TRRS cable. The microcontroller processes the data
and emulates a normal mouse. The computer can also toggle the mouse on and off via USB serial commands.

![Photo](http://i.imgur.com/31hdugM.jpg)

This repo contains the Teensyduino program used by the Teensy LC microcontroller.

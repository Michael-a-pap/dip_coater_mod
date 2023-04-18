# dip_coater_mod
This is a simple project/modification on a dip-coater in order to be able to switch solutions without the need of human intervention. I am utilizing it in order to develop solution processed Distributed Bragg Reflectors which require a stack of up to 12 layers.
The projects consist of an Arduino Nano board and a stepper motor with linear stage repurposed from an old CD/DVD drive. The on top of the linear stage is attached a solution holding vessel made from a Teflon cylinder along with a heater in order to anneal the sample. 

Currently the code is runs A non-blocking PID code with a very simplistic motion control. All of this is done in order for a user to be able to change parameters such as distance, time, temperature etc from serial but is not yet implemented. Currently, in order to change such parameters, you need to insert them and recompile the code.

The Stepper motor driver is the EasyDriver from [SparkFun](https://www.sparkfun.com/products/12779) that utilizes the A3967 IC. Furthermore, the temperature sensor for the heater is the DS18b20 1-wire temperature sensor. Libraries needs to be installed in order to be able to compile the code. 

Below is the representation of the schematic. 

![alt text](https://github.com/Michael-a-pap/dip_coater_mod/blob/main/schematic.png)

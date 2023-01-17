# Motor testing

This folder contains the scripts used for testing the old motors and motor controllers by Nanotec.

Nanolib Library: https://de.nanotec.com/produkte/9985-nanolib

Motor controller: https://en.nanotec.com/products/1771-cl3-e-2-0f

The motor controllers were connected using a Micro-USB cable with the Raspberry Pi 4.

## Setup and usage

* install nanolib. Using the  documentation available in the zip file. 
* To install Nanolib use pip with the following bash script:
- Linux:
  ```bash
  pip3 install PATH_TO_NANOTEC_LIB_ZIP/nanoteclib-N.N.N.zip
  ```
* connect the motor and the motor controller
* check connection with `nanolib_example.py`
* check if the motor works using `test_motors.py` (adjust the bus and device index if they were not 0 in the previous step)

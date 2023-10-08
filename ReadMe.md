# Overview
## Design
The current rover is equipped with four brushless DC (BLDC) motors for driving and four stepper motors for steering, as shown in the table below. 
Communication to the driving motors is done via CAN bus, while the steering motors use direct GPIO signal input. 
Furthermore, the ROS2 Foxy has been leveraged for communication of the motors with the main rover OBC and to quickly respond to the feedback from sensor data.

# Hardware Setup
## List of Components
|Sub-System|Type|P/N|Count|Comment|
|-|-|-|-|-|
|Driving Control|BLDC Motor|PD4|4|
|Driving Control|Controller|CL3-E-2-0F|4|
|Steering Control|Stepper Motor|17E1K-07|4|
|Steering Control|Closed-Loop Stepper Driver|CL57T|4|
|CAN Comms|CAN Bus|SN65HVD230|1|
|Voltage Stepper|3.3V-5V||1|

## Wiring
The figure below depicts how to wire the Jetson to the driving motor via a generic CAN bus, however, this can also be replaced with an existing PCB. For connections from the Nanotec controller to the [CL3-E Manual](https://en.nanotec.com/fileadmin/files/Handbuecher/Motorsteuerungen/CL3-E/fir-v2213/CL3E_CANopen_USB_ModbusRTU_Technical-Manual_V3.4.0.pdf?1662975689).
![[BLDC.png]]
The wiring for the stepper driver (steering motor) to the Jetson is depicted below.
![[Figures/Stepper.png]]
![[Figures/Stepper_Driver.png]]
## Operation
After connecting all the equipment, simply run the following commands in Terminal 1:
```bash
$ cd Desktop/motor_ws 
$ source install/setup.bash 
$ ros2 launch motor_control motor_run
```
In a second Terminal, run the following command:
```bash
$ ros2 launch joy joy_node
```
Assuming you're using an XBox controller, now you should be able to start the motors by pressing the "A" button. Drive forward and backward using the D-Pad "Up" and "Down". 
To turn in place press the "B" button and wait until the steering have stopped turning, now use the D-Pad "Up" and "Down" to turn clockwise and counter-clockwise. Press "B" to switch between the driving and turning modes.
# CAN Communication
The CAN bus is connected to the motor controller, which translates the CAN messages into commands that the motor can executed. All controllers are connected in parallel to the CAN bus device, to allow control of all the motors through a single port. Each motor is assigned a node ID in the network and enables direct communication for operation. The driving motors are operated in velocity mode, by directly specifying the rpm speed of the motors.
## CAN: Jetson <-> Controller
- Ensure terminator switch on controller is set to "__ON__"
- Check baudrate value of controller, by connecting it via USB
	- Check "__cfg__" file, the value should be in hexadecimal
- The bitrate must be set accordingly when activating the CAN interface
	- bitrate = baudrate
## Enable CAN Communication
To enable the CAN communication, use the following commands. A more detailed explanation of the purpose of each can be found in this [link](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9) or from [NVIDIA](https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html?).
```bash
$ sudo apt-get install busybox
$ sudo busybox devmem 0x0c303000 32 0x0000C400  
$ sudo busybox devmem 0x0c303008 32 0x0000C458  
$ sudo busybox devmem 0x0c303010 32 0x0000C400  
$ sudo busybox devmem 0x0c303018 32 0x0000C458
$ sudo modprobe can  
$ sudo modprobe can_raw  
$ sudo modprobe mttcan
```
After setting up the CAN bus, we have to configure the baudrate (i.e. 500000) to match with that of the controller, otherwise you will experience port saturation when the connection is established.
```bash
sudo ip link set can0 type can bitrate 500000
```
## Test Connection
In the first terminal we will dump all incoming messages, by using the following command:
```bash
$ candump can0
```
In the second terminal, we will send our CAN message, by using the following command:
```bash
$ sudo ip link set can0 up
$ cansend can0 607#40.4160.00.00.00.00.00
```
You should see the following response in the first terminal:
- 507:6041600000000000
Alternatively, you can use the tool __wireshark__ to more easily review the CAN packages sent and received from the Nanotec controller.
## Message Breakdown
The message structure for CAN communication is provided below. Note: Each portion of the message is set using LSB (Least Significant Byte).
XXX # (AA) - (BB BB) - (CC) - (DD DD) - (00 00)
- XXX: Node ID
- AA: Transmission Method/Length of Data (SDO)
	- Writing Code
		- 2F (1 byte)
		- 2B (2 bytes)
		- 27 (3 bytes)
		- 23 (4 bytes)
	- Reading Code
		- 40 (Request Message)
	- Response Code
		- 80 (Error message)
		- 60 (Writing Successful)
		- 4b (Reading Successful)
- BB BB: Object index 6041h is transmitted as 41 60 (= BB AA). Must be set from right to left
	- 6040 (UNSIGNED16): Cia402 Power State Machine
	- 6041 (UNSIGNED16): Cia402 Power State Machine Status
	- 6042 (INTEGER16): Vl Target Velocity Mode
	- 6060 (INTEGER8): Modes of Operation
	- 6061 (INTEGER8): Modes of Operation Display
- CC: Sub-Index
- DD DD: Object Value. Must be set from right to left

## Velocity Mode
### Message Structure
600 + nodeID # Length of Data : Index : Sub-Index : Data
### Command Sequence (Example)
Below is a sample sequence of commands required to operate the motor.
1. 607#2F:6060:00:02.00.00.00 -> Set to "Velocity Mode"
2. 607#2B:4260:00:C8.00.00.00 -> Set Velocity Value
3. 607#2B:4060:00:06.00.00.00 -> Enable Voltage
4. 607#2B:4060:00:07.00.00.00 -> Switched On
5. 607#2B:4060:00:0F.00.00.00 -> Enable Operation
6. 607#2B:4060:00:00.00.00.00 -> Quick Stop

# Troubleshooting
## CAN Overflow
When the baudrate set in the Jetson CAN configuration does not match that of the motor controller, the port will be saturated with error messages. In some instances, this may cause a visible lag in the system. To correct this simply match the baudrate of the Jetson and the controller. 
# Future Work
## Problems Encountered
### Steering Motors
While testing the steering motors (Nanotec), the following issues were encountered:
- Insufficient operational controllers were available to operate all motors
- Motors had faulty operations
- Encoders were damaged and provided no feedback
### GPIO Pins
The stepper motors used for steering make use of a minimum of two (2) GPIO pins (pulse, direction) for operation. The Jetson Orin is equipped with 26 GPIO pins, however, only 10 of those have the current capacity to operate the motor controller. Furthermore, 4 of those remaining pins are needed specifically for CAN bus operations. Finally, from the 6 available pins, only 3 pins were functional. Therefore, the only 24-mA was used to provide pulse for all four (4) controllers, and the two remaining were used to operate the motors in opposing directions. 
Due to the limited number of available pins, encoder feedback was unable to be collected. Therefore, initial orientation of the wheels had to be set manually, leading to errors in the rotation and drive of the wheels, which were further aggravated by the play in the wheel attachment to the motor.
## Recommendations
### ROS Control
The current motor control is done via a custom packages and nodes to control the motors, since steering was performed by in-place rotation. 
It is highly recommended to port the current system and implement a ROS2 package which can perform the logic of steering while driving, as to not require the rover to come to a full stop.
### Motor Procurement
Since budget is limited and motors incur a steep cost, it is my recommendation that the current hardware is maintained to allow other sub-systems, such as the suspension to receive more resources. Most of the known issues with the current hardware can be resolved with minor spending and for the most part it is properly operational. Therefore, improvements for motor control are suggested only on the software aspect.
### Wiring
The wires connecting the components were not very reliable and often led to faulty connections. 
# Lessons Learned
## Personal
Preparing and participating in the rover challenge helped me significantly improve my skills working with hardware (i.e. motors, electrical, CAN communication). It also allowed me to finally learn and implement ROS2, and how to read and troubleshoot errors. Overall this course, as well as, the ERC competition have been the best aspect of my enrollment in the MSE program.
## Rover Development
Now that there's a base design for the rover, it would be in the future team's best interest to select the 3-4 of the worst aspects of the rover to improve. This would focus resources on the most critical parts of the rover and allow more significant upgrades to occur every year.
## ERC'23
### Preparation
The Rover integration was finalized at ERC, which is also were most of the system tests were being conducted. This led to many problems having to be solved at the competition. In the future, there should be a hard deadline so that we can have a working system before traveling. 
### Team Work
During the competition there were various who were overlapping and clashing when performing tasks. This made some things redundant and many overlooked. 
There should be defined roles for all team members who attend the competition, apart from their specific system, such as gathering necessary tools to take to each Yard Task or inventory check. 

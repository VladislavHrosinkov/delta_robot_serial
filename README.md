This repository holds Arduino MEGA 2560 firmware for my master's thesis. The robot is actuated by three Herkulex 0101 servo motors. It receives commands from through a serial interface from a pc. These commands could be sent via putty, but it is intended to be used through a flutter gui held in the repository delta_controller, here: https://github.com/VladislavHrosinkov/delta_controller.git

The following commands are supported, to be separated by '\n' newline characters:

 phase int int int: Sets the phase of each motor in degrees 
 cartesian int int int: Sets the position of the motor in cartesian cooordinates
 learn:	The robot's motors have torque switched off and the robot sends its motor positions in coordinates specific to the motors. A path can then be entered manually.
 replay int int int: Repeat a position previously received from the learn command
 end: To be sent after learn to terminate the learning regime.

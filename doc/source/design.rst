Design
******

Specifications
==============

Airframe
--------

The chosen aircraft configuration is a tilt-rotor quadplane.
It is based on a 1.8m span RC glider modified to receive an arm on each wing.
Each arm is equipped with two motors. The front motors can tilt around the pitch axis.
The aerodynamics is pre-designed with PredimRC (Insert PredimRC link)

Hardware
--------

The embedded hardware is based on the Open Source UavDevBoard project (UDB5 board)
It is based on a PIC16 processor architecture.

Software
--------

The software is based on the MatrixPilot Open Source project. This software is capable of flying automatically a conventional aircraft. Pitch, yaw and roll axis are controlled by PI controllers. An energy balance allows to control the altitude using the pitch and motor power. A GPS receiver allows waypoint navigation.
Two flight modes are implemented:
  - normal flight (conventional horizontal flight)
  - hovering flight (to fly an aerobatic aircraft capable of torque roll manoeuvre)

To ensure the flight of the quadplane, several functionalities need to be added:
  - definition of a switching condition between normal and hovering flights
  - control of the motor tilting
  - control of quadplane attitude during hovering

Flight modes
++++++++++++

In the MatrixPilot code version ??, two flight modes are possible:
  - normal 
  - hovering

Hovering is only allowed if the parameter HOVERING_STABILIZED_MODE is set to 1.
The condition defining the switch between these two phases is modified. It is defined as follows:
  - normal -> hovering : the motor tilt angle is greater than a given threshold (TRANSITION_MOTOR_TILT)
  - hovering -> normal : the motor tilt angle is lower than a given threshold (TRANSITION_MOTOR_TILT)

Motor tilt control
++++++++++++++++++

Motor tilting is commanded by a servomotor. This servomotor has a given angular range (in general 120 deg) between minimum pwm (2000) to mximum pwm (4000). The desired angular range for motor tilting on a quadplane is 90 deg (horizontal to vertical position). In order to keep maximal accuracy, motor tilting control is built such that pwm 2000 to 4000 corresponds to the desired angular range (90 deg here). In addition, minimum angle, maximum angle, offset angle. Reversed control parameter. 

Attitude control
++++++++++++++++

pitch and roll control
integral terms

yaw control

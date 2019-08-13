.. _flight-modes:

Flight modes
############

In the MatrixPilot code version ??, two flight modes are possible:
  - normal 
  - hovering

Hovering is only allowed if the parameter HOVERING_STABILIZED_MODE is set to 1.
The condition defining the switch between these two phases is modified. It is defined as follows:
  - normal -> hovering : the motor tilt angle is greater than a given threshold (TRANSITION_MOTOR_TILT)
  - hovering -> normal : the motor tilt angle is lower than a given threshold (TRANSITION_MOTOR_TILT)

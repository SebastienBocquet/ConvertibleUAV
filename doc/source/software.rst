Software
========

The software is based on the MatrixPilot Open Source project version 4. This software is capable of flying automatically a conventional aircraft. Pitch, yaw and roll axis are controlled by PI controllers. An energy balance allows to control the altitude using the pitch and motor power. A GPS receiver allows waypoint navigation.
Two flight modes are implemented:
  - normal flight (conventional horizontal flight)
  - hovering flight (to fly an aerobatic aircraft capable of torque roll manoeuvre)

To ensure the flight of the quadplane, several functionalities need to be added:
  - increase of heartbeat rate and increase of output pwm frequency
  - definition of a switching condition between normal and hovering flights
  - control of the motor tilting
  - control of quadplane attitude during hovering

In the following, we assume that pwm inputs and outputs range between 2000 to 4000.


Increase of heartbeat rate
--------------------------

The heartbeat frequency should be at least 150 Hz.


Increase of output pwm frequency
--------------------------------

The output pwm frequency should be at least 150 Hz.


Flight modes
------------

In the MatrixPilot software, two flight modes are possible:
  - normal 
  - hovering

The condition defining the switch between these two phases is modified. It is defined as follows:
  - normal -> hovering : the motor tilt angle is greater than a given threshold (TRANSITION_MOTOR_TILT)
  - hovering -> normal : the motor tilt angle is lower than a given threshold (TRANSITION_MOTOR_TILT)


.. _motor_tilt:

Motor tilt control
------------------

Motor tilting is commanded by a servomotor. This servomotor has a given angular range (in general 120 deg) between minimum pwm (2000) to mximum pwm (4000). The desired angular range for motor tilting on a quadplane is 90 deg (horizontal to vertical position). In order to keep maximal accuracy, motor tilting control is built such that pwm 2000 to 4000 corresponds to the desired angular range (90 deg here). In addition, in order to adjust the servo displacement to the tilting mechanism, a minimum angle, maximum angle, and offset angle are used. The offset value determines the servo position at neutral pwm (3000). Then the minimum and maximum angles determine how much the servo is allowed to displace around the offset angle. Finally, a reversed control parameter allow to reverse the servo displacement direction.

$motor\_tilt\_pwm = (pwm\_tilt\_input-3000) \frac{MOTOR\_TILT\_SERVO\_THROW}{MOTOR\_TILT\_SERVO\_RANGE} + pwm\_centred$

The final output pwm control is:

$output\_tilt\_pwm = 3000 + REVERSE\_TILT\_CONTROL * motor\_tilt\_pwm$

The hovering mode is activated if $motor\_pitch\_pwm > pwm\_centred$


Attitude control
----------------

.. toctree::
   :maxdepth: 2

   pid
   motor_control

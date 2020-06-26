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


Conventions
-----------

The raw pwm inputs and the final pwm outputs range between $[2000; 4000]$.
However, computations are performed in practice on scaled pwm values such that neutral position corresponds to zero pwm. 
In the following, the pwm values that will be manipulated are scaled to range between $[-1000; 1000]$ for all channels except for the motor throttle which ranges between $[0; 2000]$. 

Subscript $_{eq}$ denotes the equilibrium state of the tricopter (zero forces and moments).
Subscript $_{usr}$ denotes a user input.


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


.. attitude_control::

Attitude control
----------------

.. toctree::
   :maxdepth: 2

   pid
   motor_control

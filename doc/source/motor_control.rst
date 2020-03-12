Motor control
=============

Once the roll\_quad\_control, pitch\_quad\_control and yaw\_quad\_control are computed, the final pwm outputs sent to each motor's ESC are computed as follows.
Note that the following motor control allow to obtain the same control moment for the three following motor configurations. 


Tricopter
---------

For pitch and roll control:

Considering that the throttle stick controls the variable $th_{offset}$, pwm outputs are:

  - 2000 if $th_{offset} - 3000 < MIN\_THROTTLE$, with $MIN\_THROTTLE = 0.2 * 2000$
  - if $th_{offset} - 3000 >= MIN\_THROTTLE$:

    * $th_{{control}_A} = \frac{R}{2*R_A*cos(\alpha)*(1-\beta)+R_B}*pitch\_quad\_control - \frac{R}{R_A*sin(\alpha)}*roll\_quad\_control$
    * $th_{{control}_B} = -2*\frac{R}{2*R_A*cos(\alpha)*(1-\beta)+R_B}*pitch\_quad\_control$
    * $th_{{control}_C} = \frac{R}{2*R_A*cos(\alpha)*(1-\beta)+R_B}*pitch\_quad\_control + \frac{R}{R_A*sin(\alpha)}*roll\_quad\_control$

    * $motor\_A = th_{offset} + th_{{control}_A}$
    * $motor\_B = th_{offset} + th_{{control}_B}$
    * $motor\_C = th_{offset} + th_{{control}_C}$

For yaw control:

The relationship between the angle $\beta$ in rad and the tilt pwm order is: 

.. math:: \beta = \frac{\pi}{180} motor\_tilt\_pwm \frac{MOTOR\_PITCH\_SERVO\_RANGE}{1000}
   :label: eq_tri_beta_pwm

So, the tilt motor pwm control which ensures zero yaw moment at equilibrium is :

.. math:: motor\_tilt\_pwm\_eq = \beta_{eq} \frac{180}{\pi} \frac{1000}{MOTOR\_PITCH\_SERVO\_RANGE}
   :label: eq_tri_tilt_eq_pwm

The relationship between the yaw control PID output and the angle $\beta$ remains to be determined. It is this relationship which controls the control authority around the yaw axis. We pose: 

.. math:: motor\_tilt\_pwm = K_{tilt} * yaw\_quad\_control
   :label: eq_tri_tilt_pwm

We must determined $K_{tilt}$ such that a given yaw control order creates the same yaw moment as for the quadcopter configuration. We imose the equality between :math:numref:`eq_quad_myaw` and :math:numref:`eq_tri_myaw`, and using :math:numref:`eq_tri_beta_pwm`, we obtain: $K_{tilt} = \frac{-4 K_m K_1}{2*F_{eq_A}*R_A*sin(\alpha)} \frac{180}{\pi} \frac{1000}{MOTOR\_PITCH\_SERVO\_RANGE}$.

:math:numref:`eq_tri_tilt_pwm` and :math:numref:`eq_tri_tilt_eq_pwm` entirely define the control of the yaw axisby tilting the two front motors. It allows to obtain the same yaw moment as for the equivalent quadcopter configuration for the same yaw control order. So the same PID gains should lead to the same authority around the yaw axis compared to the quadcopter configuration.

$motor\_tilt\_pwm$ is then added to $motor\_pitch\_pwm$ (see :ref:`motor_tilt`).

The coefficient $\frac{1}{1-\beta}$ is applied to $th_{{control}_A}$ and $th_{{control}_C}$ to compensate for the decrease of vertical thrust due to motor tilting.

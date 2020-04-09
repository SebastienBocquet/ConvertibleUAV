Motor control
=============

Once the roll\_quad\_control, pitch\_quad\_control and yaw\_quad\_control are computed, the final pwm outputs sent to each motor's ESC are computed as follows.
Note that the following motor control allow to obtain the same control moment for the three following motor configurations.


Tricopter
---------

For pitch and roll control:

Considering that the throttle stick controls the variable $th_{offset}$, pwm outputs are:

  * $motor\_A = th_{{eq}_A} + th_{{control}_A}$
  * $motor\_B = th_{{eq}_B} + th_{{control}_B}$
  * $motor\_C = th_{{eq}_C} + th_{{control}_C}$

The manually controlled throttle is called $th_{man}$. We pose that it is the
average value of the three motor throttles at equilibrium : $th_{man} =
\frac{1}{3} (2*th_{{eq}_A} + th_{{eq}_B})$. From
:math:numref:`eq_tri_equil_pitch` and $T = K_T*th^2$, we obtain:

  * $th_{{eq}_A} = \frac{3}{2+\sqrt{k}} th_{man}$ with $k =
    \frac{2*R_A*cos(\alpha)}{R_B}$
  * $th_{{eq}_B} = \frac{3\sqrt{k}}{2+\sqrt{k}} th_{man}$
  * $th_{{eq}_C} = th_{{eq}_A}$

and from :ref:`tri_attitude_control`

  * $th_{{control}_A} = \frac{R}{2*R_A*cos(\alpha)*(1-\beta)+R_B}*pitch\_quad\_control - \frac{R}{R_A*sin(\alpha)}*roll\_quad\_control$
  * $th_{{control}_B} = -2*\frac{R}{2*R_A*cos(\alpha)*(1-\beta)+R_B}*pitch\_quad\_control$
  * $th_{{control}_C} = \frac{R}{2*R_A*cos(\alpha)*(1-\beta)+R_B}*pitch\_quad\_control + \frac{R}{R_A*sin(\alpha)}*roll\_quad\_control$


For yaw control:

The relationship between the angle $\beta$ in rad and the tilt pwm order is:

.. math:: \beta - \beta_{eq} = \frac{\pi}{180} motor\_tilt\_pwm \frac{MOTOR\_TILT\_SERVO\_RANGE}{1000}
   :label: eq_tri_beta_pwm

So, the tilt motor pwm control which ensures zero yaw moment at equilibrium is :

.. math:: motor\_tilt\_pwm\_eq = \beta_{eq} \frac{180}{\pi} \frac{1000}{MOTOR\_TILT\_SERVO\_RANGE}
   :label: eq_tri_tilt_eq_pwm

The relationship between the yaw control PID output and the angle $\beta$ remains to be determined. It is this relationship which controls the control authority around the yaw axis. We pose:

.. math:: motor\_tilt\_pwm = K_{tilt} * yaw\_quad\_control + motor\_tilt\_pwm\_eq
   :label: eq_tri_tilt_pwm

We must determined $K_{tilt}$ such that a given yaw control order creates the same yaw moment as for the quadcopter configuration. We impose the equality between :math:numref:`eq_tri_beta` and :math:numref:`eq_tri_beta_pwm`, we obtain:

.. math:: K_{tilt} = \frac{-2 K_Q K_1}{T_{eq_A}*R_A*sin(\alpha)} \frac{180}{\pi} \frac{1000}{MOTOR\_TILT\_SERVO\_RANGE}
   :label: eq_ktilt

:math:numref:`eq_tri_tilt_pwm` and :math:numref:`eq_tri_tilt_eq_pwm` entirely define the control of the yaw axis by tilting the two front motors. It allows to obtain the same yaw moment as for the equivalent quadcopter configuration for the same yaw control order. So the same PID gains should lead to the same authority around the yaw axis compared to the quadcopter configuration.

$motor\_tilt\_pwm$ is then added to :math:numref:`eq_manual_tilt` as follows:

  - $+REVERSE\_TILT\_CONTROL * motor\_tilt\_pwm$ for the servo controlling the left motor
  - $-REVERSE\_TILT\_CONTROL * motor\_tilt\_pwm$ for the servo controlling the right motor

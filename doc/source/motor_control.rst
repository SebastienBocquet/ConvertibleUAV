Motor control
=============

Once the roll\_quad\_control, pitch\_quad\_control and yaw\_quad\_control are computed, the final pwm outputs sent to each motor's ESC are computed as follows.
Note that the following motor control allow to obtain the same control moment for the three following motor configurations.


Tricopter
---------

For pitch and roll control:

Considering that the throttle stick controls the variable $th_{offset}$, pwm outputs are:

  * $pwm\_A = th_{{eq}_A} + th_{{control}_A}$
  * $pwm\_B = th_{{eq}_B} + th_{{control}_B}$
  * $pwm\_C = th_{{eq}_C} + th_{{control}_C}$

with:

  * $th_{{eq}_A} = K\_A * th_{usr}$
  * $th_{{eq}_B} = K\_B * th_{usr}$
  * $th_{{eq}_C} = th_{{eq}_A}$

where $K\_A$ and $K\_B$ are obtained from :numref:`fig_th_ratio_theo`.

And from :ref:`tri_attitude_control`

  * $th_{{control}_A} = \frac{\sqrt{2} R_X}{2*R_A*cos(\alpha)*cos(\beta)+R_B}*pitch\_quad\_control - \frac{\sqrt{2} R_X}{R_A*sin(\alpha)}*roll\_quad\_control$
  * $th_{{control}_B} = -2*\frac{\sqrt{2} R_X}{2*R_A*cos(\alpha)*cos(\beta)+R_B}*pitch\_quad\_control$
  * $th_{{control}_C} = \frac{\sqrt{2} R_X}{2*R_A*cos(\alpha)*cos(\beta)+R_B}*pitch\_quad\_control + \frac{\sqrt{2} R_X}{R_A*sin(\alpha)}*roll\_quad\_control$

For yaw control:

Based on :eq:`eq_manual_tilt_angle`, the tilt pwm which ensures zero yaw moment at equilibrium is (we remove the offset pwm, so the following value must be seen as a $\delta\_pwm$:

.. math:: tilt\_eq\_pwm = \frac{2000*TILT\_THROW\_RATIO}{TILT\_MAX\_ANGLE\_DEG - TILT\_MIN\_ANGLE\_DEG} \beta_{eq} \frac{180}{\pi}
   :label: eq_tilt_eq_pwm

The relationship between the yaw control PID output and the angle $\beta$ remains to be determined. It is this relationship which controls the control authority around the yaw axis. We pose:

.. math:: tilt\_control\_pwm = K_{tilt} * yaw\_quad\_control
   :label: eq_tilt_pwm_for_yaw

We must determined $K_{tilt}$ such that a given yaw control order creates the same yaw moment as for the quadcopter configuration. We impose the equality between :math:numref:`eq_manual_tilt_angle` and :math:numref:`eq_tilt_pwm_for_yaw`, we obtain:

.. math:: K_{tilt} = \frac{-2 K_Q K_1}{T_{eq_A}*R_A*sin(\alpha)} \frac{180}{\pi} \frac{2000 TILT\_THROW\_RATIO}{TILT\_MAX\_ANGLE\_DEG - TILT\_MIN\_ANGLE\_DEG}
   :label: eq_ktilt

:math:numref:`eq_tilt_pwm_for_yaw` and :math:numref:`eq_tilt_eq_pwm` entirely define the control of the yaw axis by tilting the two front motors. It allows to obtain the same yaw moment as for the equivalent quadcopter configuration for the same yaw control order. So the same PID gains should lead to the same authority around the yaw axis compared to the quadcopter configuration.

Then, only in hovering mode, we add the following term to :math:numref:`eq_manual_tilt_pwm`:

  - $+REVERSE\_TILT\_CONTROL * (tilt\_control\_pwm + tilt\_eq\_pwm)$ for the servo controlling the left motor
  - $-REVERSE\_TILT\_CONTROL * (tilt\_control\_pwm + tilt\_eq\_pwm)$ for the servo controlling the right motor

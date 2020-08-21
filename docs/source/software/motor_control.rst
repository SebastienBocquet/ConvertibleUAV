Motor control
=============

.. _motor_tilt:

Motor tilt control
------------------

.. _fig_tilt_angle:
.. figure:: ../figs/motor_tilt.png
   :scale: 50 %

   Motor tilting. $\beta$ is the tilt angle in rad. $\beta=0$ corresponds to the vertical position.

Motor tilting is commanded by a servomotor. This servomotor has a given angular range (in general $120^{\circ}$ between minimum pwm (2000) to maximum pwm (4000). The desired motor tilt angle range between $TILT\_MIN\_ANGLE\_DEG$ and $TILT\_MAX\_ANGLE\_DEG$. Note that $TILT\_MIN\_ANGLE\_DEG$ needs to be a small negative angle in order to allow yaw control (see :ref:`tri_attitude_control`), so the tilt angular range is around $100^{\circ}$. In order to keep maximal accuracy, we impose that the anglular range corrresponds to the full pwm range $[2000;4000]$. In addition to $TILT\_MIN\_ANGLE\_DEG$ and $TILT\_MAX\_ANGLE\_DEG$, we introduce:

  * a coefficient $TILT\_THROW\_RATIO$ to control the angular amplitude of the servomotor
  * a reversed control parameter allows to control the servo displacement direction
  * a $TRANSITION\_ANGLE$ in radian that controls the switch between the hovering and the normal mode

.. math:: tilt\_pwm = (input\_pwm\_tilt-3000) TILT\_THROW\_RATIO
   :label: eq_manual_tilt

The final output pwm control is:

$output\_tilt\_pwm = 3000 + REVERSE\_TILT\_CONTROL * tilt\_pwm$

The relationship between the tilt angle $\beta$ and the tilt pwm is:

.. math::
  \beta \frac{180}{\pi} = \frac{TILT\_MAX\_ANGLE\_DEG - TILT\_MIN\_ANGLE\_DEG}{2000*TILT\_TRHOW\_RATIO} tilt\_pwm + \frac{TILT\_MAX\_ANGLE\_DEG + TILT\_MIN\_ANGLE\_DEG}{2}
  :label: eq_manual_tilt_angle

and conversely:

.. math::
  tilt\_pwm = \frac{1000*TILT\_THROW\_RATIO}{TILT\_MAX\_ANGLE\_DEG - TILT\_MIN\_ANGLE\_DEG} (2 \beta \frac{180}{\pi}- TILT\_MIN\_ANGLE\_DEG - TILT\_MAX\_ANGLE\_DEG)
  :label: eq_manual_tilt_pwm

The tilt pwm output corresponding to $\beta=0$ is:

.. math::
  output\_tilt\_pwm_0 = 3000 - 1000 * TILT\_THROW\_RATIO \frac{TILT\_MIN\_ANGLE\_DEG+TILT\_MAX\_ANGLE\_DEG}{TILT\_MAX\_ANGLE\_DEG-TILT\_MIN\_ANGLE\_DEG}

The hovering mode is activated if $tilt\_pwm > tilt\_pwm\_transition$, the latter being determined from :eq:`eq_manual_tilt_pwm` with $\beta=TRANSITION\_ANGLE$.


Once the roll\_quad\_control, pitch\_quad\_control and yaw\_quad\_control are computed, the final pwm outputs sent to each motor's ESC are computed as follows.
Note that the following motor control allow to obtain the same control moment for the three following motor configurations.


Motor throttle control
----------------------

For pitch and roll control:

Considering that the throttle stick controls the variable $th_{usr}$, pwm outputs are:

  .. math::
    pwm\_A = th_{{eq}_A} + th_{{control}_A} \\
    pwm\_B = th_{{eq}_B} + th_{{control}_B} \\
    pwm\_C = th_{{eq}_C} + th_{{control}_C} \\
    with: \\
    th_{{eq}_A} = K\_A * th_{usr} \\
    th_{{eq}_B} = K\_B * th_{usr} \\
    th_{{eq}_C} = th_{{eq}_A} \\
    :label: eq_throttle_eq

where $K\_A$ and $K\_B$ are obtained from :numref:`fig_th_ratio_theo`.
$th_{{control}_i}$ are defined in :math:numref:`eq_throttle_dyn`.


Throttle limiter
""""""""""""""""

$pwm\_i$ can be larger than $th_{usr}$. Since $th_{usr}$ can range between 0 and 2000, $pwm\_i$ values can exceed 2000, which is not possible (they will be cut-off to 2000). In this case, the equilibrium equations :math:numref:`eq_throttle_eq` may not be fulfilled, which can lead to dangerous situation because the UAV is not at mechanical equilibrium.
As a result, $pwm\_i$ must be limited to values below 2000, while verifying :math:numref:`eq_throttle_eq`.

We introduce a maximum throttle value which can be user-defined, for example in terms of a ratio of the max throttle range (2000): $th_{max} = th\_max\_ratio * 2000$.
If $pwm\_A$ is larger than $pwm\_B$ (because motor A is placed closer (along the x axis) to the center of gravity than motor B), we can limit throttle A to the maximum allowed value, while applying a correction on throttle B:

.. code-block:: c

  err_A = th_{max} - motor_A
  corr_B = (motor_A + err_A) * (K_B / K_A) - motor_B
  motor_A = th_{max}
  motor_B += corr_B
  motor_C = motor_A


Control around equilibrium
""""""""""""""""""""""""""

From :ref:`tri_attitude_control`

  .. math::
    th_{{control}_A} = \frac{\sqrt{2} R_X}{2*R_A*cos(\alpha)*cos(\beta)+R_B}*pitch\_quad\_control - \frac{\sqrt{2} R_X}{R_A*sin(\alpha)}*roll\_quad\_control \\
    th_{{control}_B} = -2*\frac{\sqrt{2} R_X}{2*R_A*cos(\alpha)*cos(\beta)+R_B}*pitch\_quad\_control \\
    th_{{control}_C} = \frac{\sqrt{2} R_X}{2*R_A*cos(\alpha)*cos(\beta)+R_B}*pitch\_quad\_control + \frac{\sqrt{2} R_X}{R_A*sin(\alpha)}*roll\_quad\_control \\
    :label: eq_throttle_dyn

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

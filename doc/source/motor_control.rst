Motor control
=============

Once the roll\_quad\_control, pitch\_quad\_control and yaw\_quad\_control are computed, the final pwm outputs sent to each motor's ESC are computed as follows.
Note that the following motor control allow to obtain the same control moment for the three following motor configurations. 


Quadcopter +
------------

.. figure:: figs/quadcopter_plus.png
   :scale: 100 %

   Quadcopter + configuration.


For a + quadcopter configuration with an arm of length R, the roll (or pitch) moment is:

$M_{roll} = R*(F_B - F_D)$ with $F_B$ the thrust of motor B.
$F = K*th^2$ with $K$ a coefficient depending on the propulsion chain (battery, ESC, motor and the propeller), and $th$ the throttle applied to the motor.
Considering a small throttle control around a throttle offset:
$th = th_{offset} + th_{control}$, 
then $F \approx K*2*th_{offset}*th_{control} = K_1*th_{control}$.

Finally, $M_{roll} = 2*R*K_1*th_{{control}_B}$ and $th_{{control}_D} = -th_{{control}_B}$.

We can compute pitch and roll control in the quadplane body frame are computed as follows:

  - :math:`pitch\_body\_frame\_control = pitch\_quad\_control`
  - :math:`roll\_body\_frame\_control = roll\_quad\_control`

And considering that the throttle stick controls the variable pwm\_manual\_input, pwm outputs are:

  - 2000 if pwm\_manual\_input - 3000 < MIN_THROTTLE, with MIN_THROTTLE = 0.2 * 2000
  - if pwm\_manual\_input - 3000 >= MIN_THROTTLE:

    * :math:`motor\_A = pwm\_manual\_input + yaw\_quad\_control - pitch\_body\_frame\_control`
    * :math:`motor\_B = pwm\_manual\_input - yaw\_quad\_control - roll\_body\_frame\_control`
    * :math:`motor\_C = pwm\_manual\_input + yaw\_quad\_control + pitch\_body\_frame\_control`
    * :math:`motor\_D = pwm\_manual\_input - yaw\_quad\_control + roll\_body\_frame\_control`

For such a control, $M_{roll} = 2*R*K_1*roll\_body\_frame\_control = 2*R*K_1*roll\_quad\_control$.


Quadcopter X
------------

.. figure:: figs/quadcopter_x.png
   :scale: 100 %

   Quadcopter X configuration.

For an X quadcopter configuration with an arm of length R, the roll (or pitch) moment is:

$M_{roll} = \frac{1}{\sqrt{2}}*R*(F_A + F_B - F_C - F_D$.
$F = K*th^2$ with $K$ a coefficient depending on the propulsion chain (battery, ESC, motor and the propeller), and $th$ the throttle applied to the motor.
Considering a small throttle control around a throttle offset:
$th = th_{offset} + th_{control}$, 
then $F \approx K*2*th_{offset}*th_{control} = K_1*th_{control}$.

Finally, $M_{roll} = 4*\frac{1}{\sqrt{2}}*R*K_1*th_{{control}_A} = 2*\frac{2}{\sqrt{2}}*R*K_1*th_{{control}_A}$ and $th_{{control}_B} = th_{{control}_A}$, $th_{{control}_C} = -th_{{control}_A}$, $th_{{control}_D} = -th_{{control}_A}$.

First pitch and roll control in the quadplane body frame are computed:

  - :math:`pitch\_body\_frame\_control = 0.707*(pitch\_quad\_control - roll\_quad\_control)`
  - :math:`roll\_body\_frame\_control = 0.707*(pitch\_quad\_control + roll\_quad\_control)`

Considering that the throttle stick controls the variable pwm\_manual\_input, pwm outputs are:

  - 2000 if pwm\_manual\_input - 3000 < MIN_THROTTLE, with MIN_THROTTLE = 0.2 * 2000
  - if pwm\_manual\_input - 3000 >= MIN_THROTTLE:

    * :math:`motor\_A = pwm\_manual\_input + yaw\_quad\_control - pitch\_body\_frame\_control`
    * :math:`motor\_B = pwm\_manual\_input - yaw\_quad\_control - roll\_body\_frame\_control`
    * :math:`motor\_C = pwm\_manual\_input + yaw\_quad\_control + pitch\_body\_frame\_control`
    * :math:`motor\_D = pwm\_manual\_input - yaw\_quad\_control + roll\_body\_frame\_control`

For such a control, $M_{roll} = 2*\frac{2}{\sqrt{2}}*R*K_1*roll\_body\_frame\_control = 2*\frac{2}{\sqrt{2}}*R*K_1*0.707*roll\_quad\_control = 2*R*K_1*roll\_quad\_control$.
So thanks to the $0.707$ coefficient, we obtain the same moment as for the quadcopter + configuration.

The advantage of this is that the same PID gains ensure the same control force for both configurations. Provided that the mass and mass distribution between the two configurations are close, the same stability will be obtained for both configurations keeping the PID gains. 
The main objective is to avoid or limit as much as possible PID gain tuning when changing motor configuration.


Tricopter
---------

Airframe
========

Configuration
-------------

The chosen aircraft configuration is a tilt-rotor quadplane.
It is based on a 1.8m span RC glider modified to receive an arm on each wing.
Each arm is equipped with two motors. The front motors can tilt around the pitch axis.
The aerodynamics is pre-designed with `Link PredimRC <http://rcaerolab.eklablog.com/predimrc-p1144024>`_


Mechanical model
----------------

The following notations are used:

  * $F_I = F_{eq_I} + \delta_{F_I}$ is the force produced by motor $I$, where $_{eq}$ is the value at equilibrium (the UAV is not moving) and $\delta_F$ is the value given by the attitude control.

  * $F$ is the sum of the (vertical) forces created by the motors.
  
  * $M$ is the sum of the moments created by the motors.


Hovering
^^^^^^^^

.. figure:: figs/tricopter.png
   :scale: 100 %

   Tricopter configuration.

We consider a tricopter configuration with a front arm of length $R_A$ and a rear arm length $R_B$.
The center of gravity is at the intersection of the arms.
Propeller A is CCW, propeller B is CCW and propeller C is CW.
Since there are only three motors, the yaw moment cannot be controlled by a difference in motor rotation velocity. A tilt mechanism is necessary. 
Here, the yaw moment is applied by tilting in the opposite direction the two front motors.
We call $\beta$ the tilt angle of a front motor with the vertical axis. $\beta$ positive means the motor tilts forward.

At equilibrium:

  * $F = F_{eq_A} + F_{eq_B} + F_{eq_C} = m*g$

  * The roll moment is: $M_{roll} = R_A*sin(\alpha)*(-F_{eq_A}+F_{eq_C}) = 0$
  
  * The pitch moment is: $M_{pitch} = 2*R_A*cos(\alpha)*F_{eq_A} - R_B*F_{eq_B} = 0$

  * The yaw moment is, for $\beta=0$: $M_{yaw} = -Mz_{eq_A} - Mz_{eq_B} + Mz_{eq_C}
    = K_m * (-F_{eq_A} - F_{eq_B} + F_{eq_C})$

Note that there are four equations and only three unknow forces.
So the yaw moment cannot be imposed to zero if the vertical,
pitch and roll moment equilibrium are imposed. With this choice, we obtain:

  * $F_{eq_A} = F_{eq_C}$

  * 
    .. math:: F_{eq_B} = \frac{2*R_A*cos(\alpha)}{R_B}*F_{eq_A}
       :label: eq_tri_equil_pitch

  * $2*(1 + \frac{R_A*cos(\alpha)}{R_B})*F_{eq_A} = m*g$

  * $M_{yaw} = 0 = -K_m * F_{eq_B} + sin(\beta_{eq})*(F_{eq_A}+F_{eq_C})*R_A*sin(\alpha) = -K_m * F_{eq_B} + sin(\beta_{eq})*2*F_{eq_A}*R_A*sin(\alpha)$. For small $\beta$, $sin(\beta) \approx \beta$. Finally, $M_{yaw} = 0 = -K_m * F_{eq_B} + \beta*2*F_{eq_A}*R_A*sin(\alpha)$. Using :math:numref:`eq_tri_equil_pitch`, it leads to: $\beta_{eq} = \frac{K_m cos(\alpha)}{R_B sin(\alpha)}$.

Then, for pitch and roll controls:

  * $M_{roll} = R_A*sin(\alpha)*cos(\beta)*(-\delta_{F_A} + \delta_{F_C})$.
    To ensure a constant thrust, we impose that $\delta_{F_A} = -\delta_{F_C}$.
    We also would like the roll moment to be equal to a quadcopter configuration of arm length R:

    .. math:: M_{roll} = -2*R*K_1*roll\_quad\_control
       :label: eq_roll_equiv_r
       
    with $R$ being the tricopter averaged arm length $R = \frac{1}{3}*(2*R_A+R_B)$.

    Thus, we can pose: $M_{roll} = 2*R_A*sin(\alpha)*cos(\beta)*K_1*th_{{control}_A}$, with $th_{{control}_A} = -K_{roll}*roll\_quad\_control$, $th_{{control}_C} = -th_{{control}_A}$, $th_{{control}_B} = 0$ 
    and $K_{roll} = \frac{R}{R_A*sin(\alpha)}$, which allows to fulfill equation :math:numref:`eq_roll_equiv_r`. For small $\beta$, $cos(\beta) \approx 1-\beta$.

  * $M_{pitch} = 2*R_A*cos(\alpha)*cos(\beta)*\delta_{F_A} - R_B*\delta_{F_B}$.
    To ensure a constant thrust, we impose that $\delta_{F_B} = -2*\delta_{F_A}$.
    We also would like the pitch moment to be equal to a quadcopter configuration of arm length R:

    .. math:: M_{pitch} = 2*R*K_1*pitch\_quad\_control
       :label: eq_pitch_equiv_r

    Thus, $M_{pitch} = 2*(R_A*cos(\alpha)*cos(\beta) + R_B)*K_1*th_{{control}_A}$ with $th_{{control}_A} = K_{pitch}*pitch\_quad\_control$, $th_{{control}_B} = -2*th_{{control}_A}$, $th_{{control}_C} = th_{{control}_A}$ and $K_{pitch} = \frac{R}{2*(R_A*cos(\alpha)*cos(\beta)+R_B)}$, which fulfills equation :math:numref:`eq_pitch_equiv_r`. For small $\beta$, $cos(\beta) \approx 1-\beta$.

We apply yaw control by tilting the two front motors in opposite directions around the angle $\beta_{eq}$ (for which the yaw moment is zero). For small $\beta$, $M_{yaw} = \beta*(2*F_{eq_A}+\delta_{F_A}+\delta_{F_C})*R_A*sin(\alpha)$. Assuming that the control orders are small compared to the total forces ($\delta_{F_I} << F_{eq_I}$): 

.. math:: M_{yaw} = \beta*2*F_{eq_A}*R_A*sin(\alpha)
   :label: eq_tri_myaw



Transition
^^^^^^^^^^

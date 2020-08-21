.. _quad_x_attitude_control:

Quadcopter X
------------

.. figure:: figs/quadcopter_x.png
   :scale: 100 %

   Quadcopter X configuration.


For an X quadcopter configuration motor A 
at North-East (A, B, C, D being placed anticlockwise).
The center of gravity G is at the intersection of AC and BD.
We assume that motors A and C turn a counter-clockwise (CCW) propeller, and 
motors B and D a clockwise (CW) propeller.

At equilibrium:

  * $F = T_{eq_A} + T_{eq_B} + T_{eq_C} + T_{eq_D} = m*g$

  * The roll moment is: $M_{roll} = R_R*(-T_{eq_A}-T_{eq_B}+T_{eq_C}+T_{eq_D}) = 0$.
  
  * The pitch moment is: $M_{pitch} = R_P*(T_{eq_A}+T_{eq_D}-T_{eq_B}-T_{eq_C}) = 0$

  * Concerning the yaw moment, the same relationship as for the + configuration applies:
    $-T_{eq_A} + T_{eq_B} - T_{eq_C} + T_{eq_D} = 0$

If we multiply the roll moment equation by $R_P$, and the pitch moment equation by $R_R$, and we sum the two equations, we obtain $T_{eq_B} = T_{eq_D}$.
And if we subtract them: $T_{eq_A} = T_{eq_C}$
So the same relationships as for the + configurations are obtained, leading to 
$T_{eq_A} = T_{eq_B} = T_{eq_C} = T_{eq_D}$ with $T_{eq_A} = m*g/4$ using the yaw equation.

Then for pitch and roll controls,
imposing that the attitude control has no effect on the vertical equilibrium:
($\delta_{T_A} + \delta_{T_B} + \delta_{T_C} + \delta_{T_D} = 0$):

  * $M_{roll} = R_R*(-\delta_{T_A}-\delta_{T_B}+\delta_{T_C}+\delta_{T_D})$.
    To obtain zero pitch moment, we further have $\delta_{T_A} + \delta_{T_D} - \delta_{T_B} - \delta_{T_C} = 0$. If we add with the vertical equilibrium, we obtain: $\delta_{T_D} = -\delta_{T_A}$. And if we subtract: $\delta_{T_C} = -\delta_{T_B}$. So $M_{roll} = -2*R_R*(\delta_{T_A}+\delta_{T_B})$. Further imposing zero yaw moment, we obtain $-\delta_{T_A} + \delta_{T_B} - \delta_{T_C} + \delta_{T_D} = 0$, which leads to $\delta_{T_A} = \delta_{T_B}$. Thus:

    .. math:: M_{roll} = -4*R_R*\delta_{T_A} = -4*R_R*K_1*th_{{control}_A}
      :label: eq_quadx_mroll

    with $th_{{control}_B} = th_{{control}_A}$, $th_{{control}_C} = -th_{{control}_A}$ and $th_{{control}_D} = -th_{{control}_A}

  * the same derivation for the pitch moment leads to $M_{pitch} = 4*R_P*\delta_{T_A} = 4*R_P*K_1*th_{{control}_A}$, with $th_{{control}_B} = -th_{{control}_A}$, $th_{{control}_C} = -th_{{control}_A}$ and $th_{{control}_D} = th_{{control}_A}$. 

  * 
    .. math:: M_{yaw} = -4*K_Q*K_1*th_{{control}_A}
      :label: eq_quadx_myaw
    
    with $th_{{control}_C} = th_{{control}_A}$, $th_{{control}_B} = -th_{{control}_A}$ and $th_{{control}_D} = -th_{{control}_A}$.

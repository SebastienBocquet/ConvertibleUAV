.. _lessons_learnt:

Lessons learnt
==============

Reasons for crashs
------------------

Two crashs occured for the following reasons:

  * low battery: in hovering mode, low battery is very dangerous since it leads to uncontrolled roll and pitch axis. 

  * tilt angle of the front motors: the tilt angle is controlled manually, which leads to a lot of information to process by the pilot. It happened that the front motors in forward flight mode were not completely tilted to horizontal position. They remained at a tilt angle around $70^{\circ}$ (using convention of :numref:`fig_tilt_angle`). In this case, the UAV was found impossible to control in forward flight mode.

We recommend:

  * warn in case of low battery. It required the use of a power sensor, integration of consumed energy. When $70%$ of the battery energy is consumed, the pilot must be warned. One solution is to use an on-board buzzer, or send a signal to a ground station which activates a sound signal. Note that the warning must be a clear sound signal rather than a visual warning, because the pilot will have its eyes focused on the UAV.

  * to manage the tilt angle automatically when transitioning from forward to hover mode and vice-versa. Indeed, leaving the motors in an intermediate position leads to uncontrolled UAV. Thus, we recommend to:
    
    * use a transition tilt angle lower than $30^{\circ}$, which allows to obtain a sufficient forward flight velocity for transitioning safely to forward flight mode
      
    * enforce a tilt angle of $90^{\circ}$ in forward flight mode.

    * enforce a tilt angle below $30^{\circ}$ when returning to hovering mode

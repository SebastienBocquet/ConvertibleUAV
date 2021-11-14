.. _lessons_learnt:

Lessons learnt
==============

Several dangerous situations occured due to the following:

  - low battery: I highly recommend to set with reasonable accuracy at least battery voltage measurement, and activate voice indication of battery voltage during flight on your ground control application. Indeed, during first flights with VTOLs, the transition from forward flight to hovering may not be accurately controlled, and the plane may transitions at high altitude and far from landing site. This leads to long hovering time and increases the risk of low battery.

  - aileron max angle too small. It was due to the use of fly by wire mode with too small PID coefficients. I recommend to be able to switch to MANUAL mode during all flight tests. Verify that in MANUAL mode, the full servo range is used. You may also verify that the aileron angle is sufficient in fly by wire mode. 

  .. note::

    If you control flight mode with a three-position switch from your transmitter, note that you are not limited to three flight modes. Most transmitters allow to control 6 positions by configuring a mixer on the transmitter and using an addition two-position switch.

  .. note::

    `Basic plane configuration <https://ardupilot.org/plane/docs/fpv-plane.html>`_ provides very good advices to set-up your first plane with Ardupilot.



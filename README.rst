ConvertibleUAV
==============


.. inclusion-marker-do-not-remove


.. image:: https://img.shields.io/readthedocs/pytest-executable/stable
  :target: https://convertibleuav.readthedocs.io/en/develop/?badge=develop
  :alt: Read The Docs Status   

.. image:: https://img.shields.io/github/issues/SebastienBocquet/ConvertibleUAV

.. image:: https://img.shields.io/github/license/SebastienBocquet/ConvertibleUAV

.. image:: https://img.shields.io/travis/SebastienBocquet/ConvertibleUAV/feat-ci?label=tests

Long endurance (> 1h) drones able to perform a mission autonomously while taking-off and landing in a small area are desirable for 
applications such as observation of animal species, monitoring crops or cattles in remote areas.

Current solutions include multirotors and fixed-wing drones. However, the most efficient multirotors only provide about 25min endurance, 
with a limited forward speed, leading to a limited distance range. Fixed-wing drones can provide a long range and endurance. 
Compared to multirotors or helicopters, they present the following advantages:

• More efficient flight performance 
• Able to achieve longer range mission (range increases with plane weight and aerodynamic efficiency) 
• Higher payload to weight ratio 
• Higher robustness to wind 
• Safer landing in case of engine breakdown

However, fixed-wing drones require a clear area for take-off and landing. Since the maximum range increases with drone weight and 
aerodynamic efficiency, landing becomes problematic since such drones would perform their final approach with a small slope angle and 
land with a high kinetic energy. Thus a long clear and prepared landing area is mandatory, which strongly limits the range of applications 
of such drones.

In this project we propose to prototype a solution to overcome this limitation. It is based on a tricopter tilt rotor configuration, with an embedded control software.


Documentation
-------------

In the documentation, an effort is made to describe:

  - the airframe construction choices, possible improvements and lessons learnt.

  - the mechanical equations during a fixed hovering of the UAV, allowing to determine the thrust of each propulsion, from which the throttle of each engine is deduced.
    
  - an analogy on the moments acting on the frame between a quadcopter and the tricopter. This analogy allows to obtain the same moments between a quadcopter and the tricopter, for exactly the same parameters of the control software. The benefit is that if the user has tuned a controller for a quadcopter, the same controller with the same gains can be used on the tricopter and it will give the same control stability (the moments and thus the angular accelerations for a given input control order will be the same). It avoids the tedious task of tuning the controller when changing the tricopter size or motor loations.

The mechanical equations only describe the behaviour at equilibrium and in hovering. The critical phase of transitioning from hovering to forward flight and vice-versa is not described at the moment. Indeed, it is inherently unsteady and requires a meachnical simulation tool. This work is in progress within another project. 

The documentation is accessible on `readthedocs <https://convertibleuav.readthedocs.io/en/develop/>`_.


How to build the documentation locally on Linux
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Miniconda version 2019.07 or above is required, it can be
downloaded from https://docs.conda.io/en/latest/miniconda.html

Then, execute the following commands

.. code-block:: console

  cd docs/tools/conda
  make environment
  conda activate uavDoc
  cd ../..
  make html

and open the following path in a web browser from

.. code-block:: console

  build/html/index.html


How to build the documentation locally on Windows 10
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

conda is required (version 4.8.2 only tested), but Miniconda would probably
also work (Miniconda version 2019.07 or above is required, it can be
downloaded from https://docs.conda.io/en/latest/miniconda.html)

Open a conda console
Set and activate the conda/miniconda environment : to do once by working conda session

.. code-block:: console

  cd docs
  set CONDA_CONFIG_DIR=tools/conda
  conda config --add channels conda-forge
  conda config --set channel_priority strict
  conda env create --file %CONDA_CONFIG_DIR%/environment.yml
  conda activate uavDoc
  make html

Open the generated Sphynx documentation :
Double click on :  build/html/index.html


UAV Control Software
--------------------

This control software is based on `MatrixPilot <https://github.com/MatrixPilot/MatrixPilot>`_. Follow this link to obtain the user guide, the compatible boards, and how to build and flash the code on the board.


How to run the tests
~~~~~~~~~~~~~~~~~~~~

This software can be tested on a PC prior going to fly.
Activate the simulation in the loop mode: in options.h, set SILSIM to 1.

Assuming googletest is installed:

.. code-block:: console

  cd matrixpilot
  make
  ./matrixpilot


Contributing
------------

We would be glad to receive contributions on the following points:

  - improvement of redundancy and safety, both on the frame and the software.

  - other applications than those mentioned above for such a UAV.

  - improvement of aerodynamic design for larger flight time, and better resistance to wind

  - migration of the software to `Ardupilot <https://ardupilot.org/ardupilot/>`_. Indeed, this migration will be necessary in the near future due to limitations of the UavDevBoard 5 hardware (number of output ports) limited board availability.

  - correction of errors or improvement in documentation clarity.

Please contribute through the `Github issue tracker`_. We will provide a more interactive solution on the near future.


Authors
-------

-  `Sebastien Bocquet`_ - *Project creator and maintainer*


Licence
-------

This project is released under the Apache 2.0 licence.

This documentation and software is provided without qualification standards of professional embedded softwares for aeronautical applications. Users are aware that they engage their own responsability in case of accidents and injuries.


.. _Github issue tracker: https://github.com/SebastienBocquet/ConvertibleUAV/issues    
.. _Sebastien Bocquet: https://github.com/SebastienBocquet

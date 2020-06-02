ConvertibleUAV
==============
     
:Name: ConvertibleUAV  
:Description: Prototype a VTOL tilt rotor    
:Website: https://github.com/SebastienBocquet/ConvertibleUAV.git

ConvertibleUAV

Long endurance (> 1h) drones able to perform a mission autonomously while taking-off and landing in a small area are desirable for 
applications such as: surveillance and delivery in forest, mountain or urban areas) and monitoring (crops, construction works).

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


How to generate the documentation
---------------------------------

LINUX
~~~~~

Miniconda version 2019.07 or above is required, it can be
downloaded from https://docs.conda.io/en/latest/miniconda.html

Download plantuml.jar file from `https://plantuml.com/fr/`.
In doc/source/conf.py, set the path to the plantuml.jar file on the line starting by plantuml.

Then, execute the following commands

.. code-block:: console

  cd doc
  make environment
  conda activate uavDoc
  make doc

and open the following path in a web browser from

.. code-block:: console

  build/html/index.html


WINDOWS 10
~~~~~~~~~~

conda is required (version 4.8.2 only tested), but Miniconda would probably
also work (Miniconda version 2019.07 or above is required, it can be
downloaded from https://docs.conda.io/en/latest/miniconda.html)

Open a conda console
Set and activate the conda/miniconda environment : to do once by working conda session

.. code-block:: console

  cd doc
  set CONDA_CONFIG_DIR=tools/conda
  conda config --add channels conda-forge
  conda config --set channel_priority strict
  conda env create --file %CONDA_CONFIG_DIR%/environment.yml
  conda activate uavDoc
  make -C %DOC_DIR% html

Open the generated Sphynx documentation :
Double click on :  build/html/index.html


UAV Control Software
--------------------

This control software is based on MatrixPilot `https://github.com/MatrixPilot/MatrixPilot`. Follow this link to obtain the user guide, the compatible boards, and how to build and flash the code on the board.


How to run the tests
~~~~~~~~~~~~~~~~~~~~

This software can be tested on a PC prior going to fly.
Activate the simulation in the loop mode: in options.h, set SILSIM to 1.

Assuming googletest is installed:

.. code-block:: console

  cd matrixpilot
  make
  ./matrixpilot

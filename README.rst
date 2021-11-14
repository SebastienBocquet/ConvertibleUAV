ConvertibleUAV
==============

.. inclusion-marker-do-not-remove

.. image:: https://img.shields.io/readthedocs/pytest-executable/stable
  :target: https://convertibleuav.readthedocs.io/en/develop/?badge=develop
  :alt: Read The Docs Status   

.. image:: https://img.shields.io/github/issues/SebastienBocquet/ConvertibleUAV

.. image:: https://img.shields.io/github/license/SebastienBocquet/ConvertibleUAV


This project attempts to build a VTOL aircraft with the following requirements:

  - The aircraft should make a flight consisting of a vertical take-off up to 30 m, a horizontal flight at an altitude of 60m, and a vertical landing. The landing shall be located within a prescribed square of 10m by 10m.

  - The aircraft should only use electric energy.

  - The aircraft weight should be less than 2.5kg, and the span less than 2m.

  - The horizontal flight should have a minimum duration of 15 min. Hovering time should be at least 5min.

  - In case of avorted landing, the aircraft should be able to raise at 30 m before proceeding to a second take-off.

  - The flight should be possible in presence of a maximum wind on ground of 25 km/h.

  - The flight can be semi-automatic, with manual controls from an RC emitter. A pilot with basic piloting skills should be able to make the flight.

Documentation
-------------

This documentation attempts to provide sufficient information for reproducing this project. In particular, it describes:

  - the airframe construction choices

  - the method used to determine the cruise and stall speed, as well as the required electrical power

  - the selected hardware and autopilot project used

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


Contributing
------------

We would be glad to receive contributions on the following points:

  - correction of errors or improvement in documentation clarity.

  - improvement of redundancy, reliablity and safety, both on the frame and hardware.

  - idea of applications for such an aircraft.

  - improvement of aerodynamic design for larger flight time, and better resistance to wind

Please contribute through the `Github issue tracker`_.


Authors
-------

-  `Sebastien Bocquet`_ - *Project creator and maintainer*


Licence
-------

This project is released under the Apache 2.0 licence.

This documentation and software is provided without qualification standards of professional embedded softwares for aeronautical applications. Users are aware that they engage their own responsability in case of accidents and injuries.


.. _Github issue tracker: https://github.com/SebastienBocquet/ConvertibleUAV/issues    
.. _Sebastien Bocquet: https://github.com/SebastienBocquet

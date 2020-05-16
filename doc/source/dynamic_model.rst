UAV dynamics model
==================

As explained in :ref:`transition_manoeuver`, a numerical solver is necessary to simulate the transition manoeuver. In the following, the design of this solver is detailed.

Sequence diagram
----------------

.. uml::

  @startuml
  Environment -> State: air conditions (density)
  Aircraft -> State: controls (elevator, thrust, tilt angles)
  State -> State: solve steady state
  State -> Environment: update
  State -> Dynamic_solver: velocity, attitude (angles)
  Environment -> Dynamic_solver: air conditions
  State -> System:
  System -> Dynamic_solver:
  Controls -> Dynamic_solver: user defined time-dependent controls
  Aircraft -> Dynamic_solver:
  Dynamic_solver -> Aircraft: update controls
  Aircraft -> Dynamic_solver: forces and moments
  Dynamic_solver -> Dynamic_solver: advance time
  @enduml


Class diagram
-------------

.. uml::

  @startuml
  abstract class Aircraft <|-- 

  @enduml

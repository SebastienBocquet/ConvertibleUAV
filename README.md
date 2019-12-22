# ConvertibleUAV
A convertible UAV autopilot based on the MatrixPilot - UavDevBoard project
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

In this project we propose to prototype a solution to overcome this limitation and achieve a fully autonomous flight of a fixed-wing drone, 
including vertical take-off and landing.

# Testing

To build with googletest:
  - build googletest as shared library:
    * Install gcc >= 4.8.5
    * Install CMake >= 3.6.2
    * Add C++ 11 in CMakeList file (set (CMAKE_CXX_STANDARD 11))
    * create build directory in googletest/googletest
    * cmake -DBUILD_SHARED_LIBS=ON -Dgtest_build_samples=ON -G"Unix Makefiles" ..
    * run make
    * create directories ~/usr/gtest/include and ~/usr/gtest/lib
    * cp -r ../include/gtest ~/usr/gtest/include/
    * cp lib*.so ~/usr/gtest/lib

  - build MatrixPilot with googletest:
    * activate simulation in the loop mode: in options.h, set SILSIM to 1
    * export LD_LIBRARY_PATH=/home/sbocquet/usr/gtest/lib
    * cd tests, make

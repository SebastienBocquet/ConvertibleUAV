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
    * export LD_LIBRARY_PATH=/home/sbocquet/usr/gtest/lib
    * cd tests, make

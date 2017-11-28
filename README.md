# xsensmt-yarp-driver
YARP Device Driver for XSens MT* devices based on the MT Software Suite. 

## Rationale
This repository contains the `xsensmt` YARP Device Driver (see [YARP documentation on devices](http://www.yarp.it/note_devices.html) ) that expose any Xsens-based IMU that work 
with the [Xsens MT Software Suite](https://www.xsens.com/mt-software-suite) as a C++ class with the [`IGenericSensor`](http://www.yarp.it/classyarp_1_1dev_1_1IGenericSensor.html) interface.
There is currently no precisly defined format to expose IMU devices in YARP (see https://github.com/robotology/yarp/issues/802), and so this devices tries to match 
as much as possible the behaviour of the [`xsensmtx` device](https://github.com/robotology/icub-main/tree/master/src/libraries/icubmod/xsensmtx), contained in the [`icub-main` repository](https://github.com/robotology/icub-main). 

## Installation

##### Dependencies
- [YARP](https://github.com/robotology/yarp)

##### Step-by-step installation
* Install YARP on your platform, following the instructions on [YARP documentation](http://www.yarp.it/install.html). 
* Compile the code in this repository using [CMake](https://cmake.org/) and your preferred compiler. See [YARP documentation on how to compile a CMake project](http://www.yarp.it/using_cmake.html).
* Install the project. You can specify the installation directory prefix using the [`CMAKE_INSTALL_PREFIX`](https://cmake.org/cmake/help/v3.0/variable/CMAKE_INSTALL_PREFIX.html) CMake option.
* Add `${CMAKE_INSTALL_PREFIX}/share/yarp` (where `${CMAKE_INSTALL_PREFIX}` needs to be substituted to the directory that you choose as the `CMAKE_INSTALL_PREFIX`) to your `YARP_DATA_DIRS` enviromental variable (for more on the `YARP_DATA_DIRS` env variable, see [YARP documentation on data directories](http://www.yarp.it/yarp_data_dirs.html) ). 
* Once you do that, you should be able to find the `xsensmt` device compiled by this repo using the command `yarp plugin xsensmt`, which should have an output similar to:
~~~
Yes, this is a YARP plugin
  * library:        CMAKE_INSTALL_PREFIX/lib/yarp/xsensmt.so
  * system version: 5
  * class name:     yarp::dev::XsensMT
  * base class:     yarp::dev::DeviceDriver
~~~
If this is not the case, there could be some problems in finding the plugin. In that case, just move yourself to the ``${CMAKE_INSTALL_PREFIX}/share/yarp` directory and launch the device from there.

##### Device use 
To launch the `xsensmt` device, you can connect the Xsens IMU (for example the MTI-300) to your Linux computer and the default configuration parameters should be sufficient to work fine.
To do so, launch the yarpserver, then on a terminal launch the device:
~~~
yarpdev --device inertial --subdevice xsensmt
~~~
This should open a YARP port `/inertial` , that you can read from the command line for example using the `yarp read` command:
~~~
yarp read ... /inertial
~~~
If you need to change the configuration parameters, check the device documentation in .
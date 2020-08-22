# CS - Library: Math
This library contains different math utility classes for e.g. multidimensional statistics and linear algebra. The code is open-source ([BSD License](LICENSE)) and has been tested under Ubuntu 16.04 with ROS Kinetic. Please note that this project is part of ongoing research and that there will be changes in the future.

## Structure
This project is divided up into the following subpackages:

* [cslibs\_math](cslibs_math/):<br>
    This package contains basic mathematical utilities.

* [cslibs\_math\_2d](cslibs_math_2d/):<br>
    This package mainly contains template instantiations for 2D space.

* [cslibs\_math\_3d](cslibs_math_3d/):<br>
    This package mainly contains template instantiations for 3D space.

* [cslibs\_math\_ros](cslibs_math_ros/):<br>
    This package contains functionality to convert ROS data types to ``cslibs_math`` types. This involves [tf](cslibs_math_ros/include/cslibs_math_ros/tf/), [geometry\_msgs](cslibs_math_ros/include/cslibs_math_ros/geometry_msgs/) and [sensor_msgs](cslibs_math_ros/include/cslibs_math_ros/sensor_msgs/).

## Usage



### Dependencies
This library depends on the following packages of our research group:

* [cslibs\_time](https://github.com/cogsys-tuebingen/cslibs_time)

### Examples
Some examples may be found in the test subdirectories and in other packages.

## Contributing
[Contribution guidelines for this project](CONTRIBUTING.md)

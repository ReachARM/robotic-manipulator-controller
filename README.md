# robotic-manipulator-controller

Robotic-Manipulator-Controller is the controlling software of a 3 degrees of freedom robotic manipulator intended to be used on an autonomous underwater vehicle. This project is part of my final project of studies. 

The requirements for this project are : 

  - ROS compatible
  - Kinematics and inverse kinematics equations calculated by hand, not by a robotic software or library.
  - Real-time control of the axis by the vehicle control system.


### Version
0.0.21

### Libraries, Tools and other tech stuff

This project uses the following libraries or tools : 

* [ROS Indigo](www.ros.org) - Indigo release of the Robot Operating System. 
* [C++] - The programming language used by this software is C++. 
    * Future iterations will migrate the code to *C++11* and *C++14* standards.
* [USB2Dynamixel](http://support.robotis.com/en/product/auxdevice/interface/usb2dxl_manual.htm) - Communication between the computer and the servos through USB using the Robotis USB2Dynamixel.
* [Orocos-KDL](http://www.orocos.org/kdl) - Kinematics and dynamics library for robotics. 
* [DXL library](http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel/usb2dxl_linux.htm) - Dynamixel SDK driver for Linux based system.
* [rViz](http://wiki.ros.org/rviz) - Vizualisation app used to test the kinematic equations.
* [URDF](http://wiki.ros.org/urdf) - XML based descriptive file of the robot model used by rViz.

You can find the project at [robotic-manipulator-controller](https://github.com/kritchie/robotic-manipulator-controller) on GitHub.

### Installation

In order to use the code to control the servos, you have to install several packages beforehand.  

Follow the installation instructions provided by ROBOTIS for :

* USB2Dynamixel : 
http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel/usb2dxl_linux.htm
* DXL SDK for Linux : http://support.robotis.com/en/baggage_files/dynamixel_sdk/dxl_sdk_linux_v1_01.zip

Readme files are included in the packages with the installation instruction from ROBOTIS.

### Todos

 - Create a test script for automatic diagnostic of the controller.
 - Comment code and add tutorial
 - Calculate kinematics equations and integrate them into the controller
 - *C++11* and *C++14* migration

License
----
MIT

### Links
* Robot Operating System : ros.org
* ROBOTIS support : http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel/usb2dxl_linux.htm

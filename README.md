 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Simple RViz plugin
=============================

# Travis CI

[![Travis-CI](https://api.travis-ci.org/InstitutMaupertuis/simple_rviz_plugin.svg?branch=indigo-devel)](https://travis-ci.org/InstitutMaupertuis/simple_rviz_plugin/branches) 

This is a ROS package of a simple RViz plugin enabling to move a Fanuc M10iA robot along it's Z axis.

<img src="https://raw.githubusercontent.com/InstitutMaupertuis/simple_rviz_plugin/indigo-devel/simple_rviz_plugin.png" align="center" height="300">

Directories in the project
--------------------------

| Directory  | Description
------------ | -----------
`simple_node` | Contains the definition and implementation of a `SendOffset` service.
`simple_rviz_plugin` | Defines a Qt widget with buttons that are connected to Qt slots, uses `SendOffset` to send a service request.

Dependencies
------------
- [Robot Operating System](http://wiki.ros.org/ROS/Installation)
- [`industrial-core`](http://wiki.ros.org/industrial_core)
- [`fanuc`](https://github.com/ros-industrial/fanuc)

This package has been tested with Ubuntu 14.04 and ROS Indigo.

Install
-------
Install the dependencies by following the wiki instructions and cloning the repositories into your catkin workspace.

`cd` to your catkin workspace source directory:
```
git clone https://github.com/InstitutMaupertuis/simple_rviz_plugin.git &&\
cd .. &&\
catkin build
```

Launching
---------
Source `devel/setup.bash` and launch:

```
roslaunch simple_node simple_node_rviz.launch
```

What happens
------------
The button `Move robot` is connected to the [moveRobotButtonHandler](https://github.com/InstitutMaupertuis/simple_rviz_plugin/blob/indigo-devel/simple_rviz_plugin/src/simple_panel.cpp#L68-L89) slot, which fills a service request and launches a thread to send that request and wait for an answer.
Note that the panel will remain freezed until the response has been received.

For more details about why this is done like that, please read [Rviz panel and message passing](https://groups.google.com/d/msg/swri-ros-pkg-dev/nKf0h1O0h1o/AeJ8_va5DwAJ).

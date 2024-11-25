# ls01b_ros

## version track
Author: leo

### ver2.01_191226 leo
1. Add angle compensate
2. Add truncate angle area data

## Description
The `ls01b_ros` package is a linux ROS driver for ls01b.
The package is tested on Ubuntu 14.04 with ROS kinetic.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
```

**Published Topics**

``/scan`` (`sensor_msgs/scan`)

**Node**

```
roslaunch ls01b_v2 ls01b_v2.launch
```

## FAQ

## Bug Report








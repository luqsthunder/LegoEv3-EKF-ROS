# Lego EV3Dev(linux) + ROS with Extended Kalman Filter

This repository is a reproduction of the [**Localization of Mobile Robots Using an Extended Kalman Filter in a LEGO NXT**](https://ieeexplore.ieee.org/document/5782966) paper using a Lego EV3 and ROS instead.

### References for Odmetry
In order to implement odmetry for Ev3Dev, one can use [Odometrium](https://github.com/sterereo/odometrium) as reference.
There is also an example of how to perform odometry in [ROS Navigation](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom) tutorials section.

### References for Kalman Filter
There is an implemented package for Kalman Filter that combines Odometry and visual information. It is called [Robot Pose EKF](http://wiki.ros.org/robot_pose_ekf).

### For further details, please check out out [Wiki](https://github.com/luqsthunder/LegoEv3-EKF-ROS/wiki).

### Pending Tasks
- [ ] add code inside EV3 to read sensors
- [ ] add description of how ultra-sound sensors measurement is different from article infra-red sensors
- [ ] add rviz code in master to read sensors measurement from lego
- [ ] create code for odometry in lego
- [ ] send odometry to ekf_robot_pose running on ros_master

# Lego EV3Dev(linux) + ROS with Extended Kalman Filter

This repository is a reproduction of the [**Localization of Mobile Robots Using an Extended Kalman Filter in a LEGO NXT**](https://ieeexplore.ieee.org/document/5782966) paper using a Lego EV3 and ROS instead.

#### Reference for Odmetry calculation using EV3Dev
In order to implement odmetry for Ev3Dev, one can use [Odometrium](https://github.com/sterereo/odometrium) as reference.

#### Reference for Kalman Filter package
There is an implemented package for Kalman Filter that combines Odometry and visual information. It is called [Robot Pose EKF](http://wiki.ros.org/robot_pose_ekf).

#### Optional, Debug server gdb from pc(ros_master node) to EV3
- need to add the steps
- cgdb can be a option to a more human interface
- clion can run gdb debug server for a more human interface than cgdb
#### Pending Tasks
- [ ] Can cross-compile from pc to lego without generate new image ?
- [ ] add code inside EV3 to read sensors
- [ ] add description of how ultra-sound sensors measurement is different from article infra-red sensors
- [ ] add rviz code in master to read sensors measurement from lego
- [ ] create code for odometry in lego
- [ ] send odometry to ekf_robot_pose running on ros_master

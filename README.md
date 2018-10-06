# Lego EV3Dev(linux) + ROS with Extended Kalman Filter
This repository is a Reproduction of papper Localization of Mobile Robots Using an Extended Kalman Filter in a LEGO NXT.
but using lego ev3 and ROS.

#### Creation EV3Dev's image with ROS_Common:
- 1 - Install docker (https://www.docker.com/)
- 2 - Download image using docker. Image is ev3dev-jessie/ev3-base (https://github.com/ev3dev/docker-library)
    - command: $ docker pull ev3dev-jessie/ev3-base
    - if you want to download other image than ev3dev-jessie / ev3-base please pay attention if image has everything, boot partition, linux-headers, kernel, etc... E.g: ev3dev/debian-jessie-cross does not boot after put in SDcard
- 3 - Run container with downloaded image 
- 4 - Install ROS's dependencies and ROS  (https://github.com/moriarty/ros-ev3/blob/master/brickstrap-build-status.md)
- 5 - Install Python3
- 6 - Install ev3dev python libraries :
  - sudo apt-get update
  - sudo apt-get install python3-ev3dev2
- 7 - Run installation of rospkg for python3 ( https://stackoverflow.com/questions/49758578/installation-guide-for-ros-kinetic-with-python-3-5-on-ubuntu-16-04)
- 8 - open another terminal, if you exit the terminal with docker, changes will not been commited
- 9 - Commit running container to a new EV3's image with ROS_common installed
(https://docs.docker.com/engine/reference/commandline/commit/#parent-command)
- 10 - Generate  .tar and  .img of new image created from previous command trougth brickstrap generate .tar and afeter  .img you can find Brickstrap here (https://github.com/ev3dev/brickstrap):
  - generate .tar command: $ sudo brickstrap/src/brickstrap.sh create-tar NAME_OF_COMMITED_NEW_IMG FILE_GENERATE_NAME.tar
  - generate .img using: $ sudo brickstrap/src/brickstrap.sh create-image FILE_GENERATE_NAME.tar FINAL_FILE.img
- 11 - Burn .img Inside SDcard (https://etcher.io/)


# Set ev3dev to ros master node outside Lego Ev3
- need to add the steps 

# Optional, Debug server gdb from pc(ros_master node) to EV3
- need to add the steps 
- cgdb can be a option to a more human interface
- clion can run gdb debug server for a more human interface than cgdb
# Pending Tasks
- [ ] Can cross-compile from pc to lego without generate new image ?
- [ ] add code inside EV3 to read sensors
- [ ] add description of how ultra-sound sensors measurement is different from article infra-red sensors
- [ ] add rviz code in master to read sensors measurement from lego
- [ ] create code for odometry in lego
- [ ] send odometry to ekf_robot_pose running on ros_master

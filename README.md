# Lego EV3Dev(linux) + ROS with Extended Kalman Filter

This repository is a reproduction of the [**Localization of Mobile Robots Using an Extended Kalman Filter in a LEGO NXT**](https://ieeexplore.ieee.org/document/5782966) paper using a Lego EV3 and ROS instead.

## Getting Started

### Prerequisites
- Install [docker](https://www.docker.com/).
- Download the following image [ev3dev-jessie/ev3-base](https://github.com/ev3dev/docker-library) using docker.
```bash
  $ docker pull ev3dev-jessie/ev3-base
```
You may download another image than ev3dev-jessie / ev3-base but please be aware
if the given image has everything such as: boot partition, linux-headers, kernel etc.
E.g: The ev3dev/debian-jessie-cross image will not boot after put in a SDcard.

- Next, run a container with the downloaded image.
- Install [ROS and its dependencies](https://github.com/moriarty/ros-ev3/blob/master/brickstrap-build-status.md)
- Install Python3
- Install ev3dev python libraries:
  ```bash
    $ sudo apt-get update
    $ sudo apt-get install python3-ev3dev2
  ```

- Run installation of [rospkg for python3](https://stackoverflow.com/questions/49758578/installation-guide-for-ros-kinetic-with-python-3-5-on-ubuntu-16-04)

#### Creating EV3Dev's image with ROS_Common:

- Firstly, open another terminal. (If you exit docker terminal changes will not been commited)
- [Commit the running container](https://docs.docker.com/engine/reference/commandline/commit/#parent-command) to a new EV3's image with ROS_common installed
- Generate a .tar and a .img of the new image created from the previous command through Brickstrap. You can find it [here](https://github.com/ev3dev/brickstrap).
  - How to generate .tar:
  ```bash
    $ sudo brickstrap/src/brickstrap.sh create-tar NAME_OF_COMMITED_NEW_IMG FILE_GENERATE_NAME.tar
  ```
  - How to generate .img
  ```bash
    $ sudo brickstrap/src/brickstrap.sh create-image FILE_GENERATE_NAME.tar FINAL_FILE.img
  ```
- And finally [burn the .img to a SDcard](https://etcher.io/)

#### Reference for Odmetry calculation using EV3Dev
In order to implement odmetry for Ev3Dev, one can use [Odometrium](https://github.com/sterereo/odometrium) as reference.

### Example of mounting a syncronized folder Lego to your PC
After pairing via Bluetooth, use the following command:
```
sudo sshfs -o  allow_other robot@ev3dev.local:/FOLDER/PATH/INSIDE/LEGO/ /FOLDER/PATH/INSIDE/HOST
```


#### TODO: Set ev3dev to ros master node outside Lego Ev3

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

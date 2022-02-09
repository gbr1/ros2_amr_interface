# ROS2 AMR interface

Lightweight package to get AMRs working with ROS2.



## Run node

`ros2 run ros2_amr_interface amr_interface_node`

if you need to change serial port:<br>
`ros2 run ros2_amr_interface amr_interface_node --ros-args -p port_name:=<your port>`


## Run teleop

`ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=~/dev_ws/src/ros2_amr_interface/config/joy.config.yaml`

## Launch file
`ros2 launch ros2_amr_interface minimal_launch.py`

## How install


### Prerequisites

- [ROS2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
- [workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
- [permissions on dialout](https://github.com/gbr1/TIL/blob/main/Linux/22-01-24_How_add_user_to_dialout_group.md)

### Install AMR interface package

You require to install dependencies, since binaries sometimes are not updated install transport_drivers from source:
``` bash
cd ~/dev_ws/src
git clone https://github.com/ros-drivers/transport_drivers.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

then install amr package by:
``` bash
cd ~/dev_ws/src
git clone https://github.com/gbr1/ros2_amr_interface.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

**NOTE:** add install/setup.bash to your .bashrc to be faster in using terminal tabs

---

## Parameters

### Serial communication

`port_name`, _std::string_, port name<br>
`timeout_connection`, _float_, how many seconds of no communication are required to declare timeout<br>
`try_reconnect`,  _bool_, force reconnection after timeout<br>
`show_extra_verbose`, _bool_, show extra verbose in terminal<br>

### IMU

`imu.frame_id`, _std::string_, frame id used for imu<br>
`imu.offset.accelerometer.x/y/z`, _float_, offset on accelerometer<br>
`imu.offset.gyro.x/y/z`, _float_, offset on gyroscope<br>
`imu.scale.accelerometer`, _float_, range of m/s^2<br>
`imu.scale.gyro`, _float_, range of rad/s<br>

### Model

`model.size.chassis.x`, _float_, lx on mecanum model<br>
`model.size.chassis.y`, _float_, ly on mecanum model<br>
`model.size.wheel.radius`, _float_, wheel radius on mecanum model<br>
`publishTF`, _bool_, broadcast the transformation for odometry<br>
`frame_id`, _std::string_, robot frame id<br>
`odom.frame_id`, _std::string_, frame id for odometry<br>

---

## ToDos
- [x] add odom
- [x] add tf
- [x] add cmd_vel
- [x] add odom reset/init pos
- [x] add imu
- [x] add battery
- [x] add parameters (device name, robot dimensions, etc)
- [x] tf broadcaster should be disabled to not cause issues with localization 
- [x] add launch file as example
- [x] create a namespace for topics (e.g. /amr/cmd_vel)
- [x] add timeout
- [ ] refactor code in files
- [ ] add Lifecycle
- [ ] add diff drive mode


> ***Copyright (c) 2022 G. Bruno under MIT license***
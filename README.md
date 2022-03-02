# ROS2 AMR interface

Lightweight package to get AMRs working with ROS2.



## Run node

`ros2 run ros2_amr_interface amr_interface_node`

if you need to change serial port and remap a topic:<br>
`ros2 run ros2_amr_interface amr_interface_node --ros-args -p port_name:=<your port> --remap /amr/cmd_vel:=/cmd_vel`


## Run teleop

`ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=</your/full/path/to>/dev_ws/src/ros2_amr_interface/config/joy.config.yaml`

## Launch file

`ros2 launch ros2_amr_interface minimal_launch.py`

## How install


### 1. Prerequisites

- [ROS2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
- [workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
- [permissions on dialout](https://github.com/gbr1/TIL/blob/main/Linux/22-01-24_How_add_user_to_dialout_group.md)

### 2. Install AMR interface package

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

## Parameters

You can learn more about node's parameters [here](./docs/parameters.md).

## Examples

If you need to use `ros2_amr_interface` with your own hardware, you can check this [guide](./docs/arduino_example.md) on how to use with arduino.
<br>
There are also some config files for rviz and joypads.

---


> ***Copyright (c) 2022 G. Bruno under MIT license***
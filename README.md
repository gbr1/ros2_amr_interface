# ROS2 AMR interface

Lightweight package to get AMRs working with ROS2.

## Launch file

`ros2 launch ros2_amr_interface minimal_launch.py`

## Nodes

- [amr_interface_node](.docs/nodes.md)

<br>
<br>

---



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

<br>
<br>

---

## Parameters

You can learn more about node's parameters [here](./docs/parameters.md).

## Topics

Check this [documentation](./docs/topics.md) about pub/sub topics.

## Examples

If you need to use `ros2_amr_interface` with your own hardware, you can check this [guide](./docs/arduino_example.md) on how to use with arduino.
<br>
There are also some config files for rviz and joypads, here an example of [how to start a joypad](./docs/joypad.md).

---


> ***Copyright (c) 2022 G. Bruno under MIT license***
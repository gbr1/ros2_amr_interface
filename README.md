# ROS2 AMR interface

Lightweight package to get AMRs working with ROS2.



## Run node

`ros2 run ros2_amr_interface amr_interface_node`

## Run teleop

`ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=/home/gbr1/dev_ws/src/ros2_amr_interface/config/joy.config.yaml`

## How install

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
git submodule init
git submodule update
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

**NOTE:** add install/setup.bash to your .bashrc to be faster in using terminal tabs


---

## ToDos
- [x] add odom
- [x] add tf
- [x] add cmd_vel
- [ ] add Lifecycle
- [ ] add imu
- [ ] add odom reset
- [ ] add battery
- [ ] refactor code in files
- [ ] add parameters (device name, robot dimensions, etc)
- [ ] add launch files
- [ ] tf broadcaster should be disabled to not cause issues with localization 
- [ ] create a namespace for topics (e.g. /amr/cmd_vel)
- [ ] add diff drive mode


> ***Copyright (c) 2022 G. Bruno under MIT license***
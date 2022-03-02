# Nodes


## amr_interface_node


This node allows to get serial communication working between hardware and ROS2. <br>
It can manage:
- joints
- odometry
- imu
- battery

<br>

To run:<br>
`ros2 run ros2_amr_interface amr_interface_node`
<br><br>
if you need to change serial port and remap a topic:<br>
`ros2 run ros2_amr_interface amr_interface_node --ros-args -p port_name:=<your port> --remap /amr/cmd_vel:=/cmd_vel`

<br>

Here a scheme of communication messages: <br>
![flow.drawio.svg](./images/flow.drawio.svg)

---


> ***Copyright (c) 2022 G. Bruno under MIT license***
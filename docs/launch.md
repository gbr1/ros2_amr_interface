# Edit launch file

Here is shown the [minimal launch](../launch/minimal_launch.py):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_amr_interface',
            executable='amr_interface_node',
            name='amr',
            remappings=[
                ('/amr/cmd_vel', '/cmd_vel')
            ],
            parameters=[{
                'try_reconnect': False
                }]
        )
    ])
```

You can edit this file to change your parameters and remaps. <br>
For example, if your need to change the serial port name, you can add in parameters `'serial_port':"<your device>"`, etc.
<br>
<br>
For all parameters and topics, please check [parameters docs](./parameters.md) and [topics docs](./topics.md).

---


> ***Copyright (c) 2022 G. Bruno under MIT license***
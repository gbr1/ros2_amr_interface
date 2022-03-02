# Parameters

After running the node, you can check parameters by:<br>
`ros2 param list`<br>
or<br>
`ros2 run rqt_reconfigure rqt_reconfigure`
<br>

---

### Serial communication

- `port_name` [ /dev/ttyUSB0 ], _std::string_, port name<br>
- `baud_rate` [ 115200 ], _int_, baud rate<br>
- `timeout_connection` [ 60.0 ], _float_, how many seconds of no communication are required to declare timeout<br>
- `try_reconnect` [ true ],  _bool_, force reconnection after timeout<br>
- `show_extra_verbose` [ false ], _bool_, show extra verbose in terminal<br>
- `publishBattery` [true], _bool_, enable or disable Battery message<br>


### IMU

- `publishIMU` [true], _bool_, enable or disable IMU <br>
- `imu.frame_id` [ imu_link ], _std::string_, frame id used for imu<br>
- `imu.offset.accelerometer.x/y/z` [ 0.0 ], _float_, offset of accelerometer measuraments on x, y and z axis<br>
- `imu.offset.gyro.x/y/z` [0.0], _float_, offset of gyroscope measuraments on x,y and z axis<br>
- `imu.scale.accelerometer` [ 2.0 ], _float_, range of m/s^2<br>
- `imu.scale.gyro` [ 250.0 ], _float_, range of rad/s<br>


### Model

- `publishTF` [ true ], _bool_, broadcast the transformation for odometry<br>
- `frame_id` [ base_link ], _std::string_, robot frame id<br>
- `odom.frame_id` [ odom ], _std::string_, frame id for odometry<br>
- `model.type` [ mecanum ], _std::string_, options: "mecanum", "differential" and "skid"<br>
    - for mecanum:<br>
        - `model.size.chassis.x` [ 0.0825 ], _float_, lx on mecanum model (half of the wheel base)
        - `model.size.chassis.y` [ 0.105 ], _float_, ly on mecanum model (half of the distance between left wheels and right wheels)
        - `model.size.wheel.radius` [ 0.04 ], _float_, wheel radius on mecanum model<br>
    - for differential:<br>
        - `model.size.chassis.wheel_separation` [0.15], _float_, distance between two wheels
        - `model.size.wheel.radius` [0.0325], _float_, wheel radius on differential model
    - for skid:<br>
        - `model.size.chassis.wheel_separation` [ 0.0825 ], _float_, half of the distance between left wheels and right wheelss
        - `model.size.wheel.radius` [ 0.04 ], _float_, wheel radius

## Dynamic parameters
- `show_extra_verbose`
- `publish_TF`
- `imu.offset.accelerometer.x/y/z`
- `imu.offset.gyro.x/y/z`
- `imu.scale.accelerometer`
- `imu.scale.gyro`

---

> ***Copyright (c) 2022 G. Bruno under MIT license***
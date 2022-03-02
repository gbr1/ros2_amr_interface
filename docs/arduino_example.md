# An Arduino example to use ros2_amr_interface

Here is provided a guide on how to write your own firmware using Arduino language.

You need to include [ucPack library](https://github.com/gbr1/ucPack) to our sketch, then to declare an ucPack class. This object will be help you to manage communication to ROS2.

We need some timers and variables.

``` c++
#include "ucPack.h"

ucPack packeter(100);
char c;
unsigned long timer_motors = 0;
unsigned long timer_joints = 0;
unsigned long timer_imu = 0;
unsigned long timer_battery = 0;
unsigned long timer_timeout = 0;


float accelerometer_x, accelerometer_y, accelerometer_z, gyro_x, gyro_y, gyro_z, temperature;

float wheel_speed_0, wheel_speed_1, wheel_speed_2, wheel_speed_3;
float w0, w1, w2, w3;

float battery_voltage = 0.0;

float timeout = 0.0;
```

In `setup` function, you must start serial communcation and initialize your hardware. Don't forget to save `millis()` in each timer.

``` c++
void setup(){
    Serial.begin(115200);

    // initialize your hardware

    timer_motors = millis();
    timer_joints = millis();
    timer_imu = millis();
    timer_battery = millis();
    timer_timeout = millis();
}
```

The `loop` function is the core of your firmware. Using the timers we create a polling system to update all the peripherials and get the communication working with ROS2.<br>
The simple idea is to check if any serial data is available and push them into the circular buffer of ucPack.
We need to check if any `payload`, or more simply understandable data, is available. <br>
Using the code you will be able to understand the type of the message and choose the correct operation (e.g. update motors reference).
``` c++
void loop(){

    // update motors speed 
    if (millis()-timer_motors>10){
        //update motor speed

        timer_motors = millis();
    }

    // send joints data to ros2 
    if (millis()-timer_joints>10){
        dim = packeter.packetC4F('j', wheel_speed_0, wheel_speed_1, wheel_speed_2, wheel_speed_3);
        timer_joints = millis();
    }

    // send imu data to ros2
    if (millis()-timer_imu>10){
        //update imu data

        dim = packeter.packetC8F('i', accelerometer_x, accelerometer_y, accelerometer_z, gyro_x, gyro_y, gyro_z, temperature, 0.0);
        timer_imu = millis();
    } 

    //send battery data to ros2
    if (millis()-timer_battery>1000){
        // read battery voltage

        dim = packeter.packetC1F('b', battery_voltage);
        Serial.write(packeter.msg, dim);
        timer_battery = millis();
    }

    // check timeout
    if (millis()-timer_timeout>timeout){
        // your connection was broken, go to a secure operation to block your hardware
    }

    // load data from serial port
    while(Serial.available>0){
        packeter.buffer.push(Serial.read());
    }

    // unpacket messages from ROS2
    while(packeter.checkPayload()){
        timer_timeout = millis();
        c=packeter.payloadTop();

        // enable robot
        if (c=='E'){
            packeter.unpacketC1F(c, timeout);
            dim = packeter.packetC1F('e', timeout);
            Serial.write(packeter.msg, dim);

            // do other stuffs that make your hardware active
        }
        
        // update joints
        if (c=='J'){
            packeter.unpacketC4F(c, w0, w1, w2, w3);
            dim = packeter.packetC1F('x', 0.0);
            Serial.write(packeter.msg, dim);

            // set motors reference speeds using w0, w1, w2, w3
        }

        // stop the robot
        if (c=='S'){
            dim = packeter.packetC1F('s', 0.0);
            Serial.write(packeter.msg, dim);
            // stop the robot
        }

        // set gyro scale if imu is connected
        if (c=='G'){
            packeter.unpacketC2F(c, accelerometer_scale, gyro_scale);

            // set imu scales and update accelerometer_scale and gyro_scale if they are fully setted

            // acknowledge ros
            dim = packeter.packetC2F('g', accelerometer_scale, gyro_scale);
            Serial.write(packeter.msg, dim);
        }
    }
}
```

The timeout is needed if you want to add a security mechanism on hardware fails. If you don't want to use it, you can set `-1` on ROS2 side.

---


> ***Copyright (c) 2022 G. Bruno under MIT license***
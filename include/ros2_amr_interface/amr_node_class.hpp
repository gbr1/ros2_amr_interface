/*
 * The MIT License
 *
 * Copyright (c) 2022 Giovanni di Dio Bruno https://gbr1.github.io
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef __AMR_NODE_CLASS_HPP__
#define __AMR_NODE_CLASS_HPP__

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "ros2_amr_interface/FIKmodel.hpp"
#include "ucPack/ucPack.h"

using namespace std::chrono_literals;



class AMR_Node: public rclcpp::Node{
    private:
        FIKmodel mecanum;
        ucPack packeter;
        double imu_offset_acc_x, imu_offset_acc_y, imu_offset_acc_z, imu_offset_gyro_x, imu_offset_gyro_y, imu_offset_gyro_z;
        float vx, vy, w, x, y, theta, ax, ay, az, gx, gy, gz;
        double dt;
        float battery;
        bool publishTF;
        float timeout_connection;
        bool connected;


        bool to_be_publish;
        bool imu_data_available;
        bool battery_data_available;

        // Required for serial communication
        std::unique_ptr<IoContext> node_ctx{};
        std::string device_name;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver;
      
        // Timers and times
        rclcpp::Time previous_time;
        rclcpp::TimerBase::SharedPtr odom_timer;
        rclcpp::TimerBase::SharedPtr imu_timer;
        rclcpp::TimerBase::SharedPtr battery_timer;
        rclcpp::TimerBase::SharedPtr connection_timer;
        
        // Subscribers and Publishers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_subscription;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr initial_pose_subscription;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bc;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher;

        OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle;



        // Callback for /joy topic subscription
        void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            std::vector<uint8_t> serial_msg;
            float w1,w2,w3,w4;
            mecanum.forward(msg->linear.x,msg->linear.y,msg->angular.z,w1,w2,w3,w4);
            uint8_t dim=packeter.packetC4F('J',w1,w2,w3,w4);
            for(uint8_t i=0; i<dim; i++){
                serial_msg.push_back(packeter.msg[i]);
            }
            serial_driver->port()->async_send(serial_msg);
            serial_msg.clear();
            RCLCPP_INFO(this->get_logger(),"sent: %f\t%f\t%f\t%f",w1,w2,w3,w4);
        }

        // Callback for /initial_pose topic subscription
        void initial_pose_callback(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg) {
            x=msg->pose.position.x;
            y=msg->pose.position.y;

            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll,pitch,yaw;
            m.getRPY(roll,pitch,yaw);
            theta=float(y);
            
            RCLCPP_INFO(this->get_logger(), "new pose:\t\t%f %f %f\t%f %f %f", x, y, 0.0, 0.0, 0.0, theta);
        }

        // Callback for serial communication
        void serial_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred){
            for (int i=0; i<int(bytes_transferred); i++){
                packeter.buffer.push(buffer[i]);
                //std::cout<<std::hex<<int(buffer[i])<<std::dec<<" ";
            }
            std::cout<<std::endl;
            
            while (packeter.checkPayload()){
                uint8_t c=packeter.payloadTop();

                if (!connected){
                    if (c=='e'){
                        connection_timer->cancel();
                        connected=true;
                    }
                }
                else{
                    if (c=='j'){
                        float w1,w2,w3,w4;
                        packeter.unpacketC4F(c,w1,w2,w3,w4);
                        rclcpp::Time now = this->get_clock()->now();
                        //---------------------------------------------------------------------------
                        RCLCPP_INFO(this->get_logger(),"joints: %f\t%f\t%f\t%f",c,w1,w2,w3,w4);
                        //---------------------------------------------------------------------------
                        mecanum.inverse(w1,w2,w3,w4,vx,vy,w);
                        //RCLCPP_INFO(this->get_logger(),"odom: %f\t%f\t%f",vx,vy,w);
                        dt=now.seconds()-previous_time.seconds();
                        previous_time=now;
                        
                        float dtheta=w*dt;
                        float dx=(vx*cos(theta)-vy*sin(theta))*dt;
                        float dy=(vx*sin(theta)+vy*cos(theta))*dt;
                        x+=dx;
                        y+=dy;
                        theta+=dtheta;
                    }
                    
                    // imu message from hardware
                    if (c=='i'){
                        float temp, f;
                        packeter.unpacketC8F(c,ax,ay,az,gx,gy,gz,temp,f);
                        //-----------------------------------------------------------------------------------------
                        RCLCPP_INFO(this->get_logger(),"imu: %f\t%f\t%f\t%f\t%f\t%f\t%f",ax,ay,az,gx,gy,gz,temp);
                        //-----------------------------------------------------------------------------------------
                        imu_data_available=true;
                    }
                
                    // battery message from hardware
                    if (c=='b'){
                        packeter.unpacketC1F(c,battery);
                        //-----------------------------------------------------------------------------------------
                        RCLCPP_INFO(this->get_logger(),"battery: %f V",battery);
                        //-----------------------------------------------------------------------------------------
                        battery_data_available=true;
                    }
                }
                // joints message from hardware

            }
        }

        // Odometry publisher and TF broadcaster
        void odom_pub_callback(){
            rclcpp::Time now = this->get_clock()->now();
            tf2::Quaternion q;
            q.setRPY(0.0,0.0,this->theta);

            if (publishTF){
                geometry_msgs::msg::TransformStamped t;

                t.header.stamp = now;
                t.header.frame_id = "odom";
                t.child_frame_id = "base_link";

                t.transform.translation.x = this->x;
                t.transform.translation.y = this->y;
                t.transform.translation.z = 0.0;

                
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                this->tf_bc->sendTransform(t);
            }
            

            nav_msgs::msg::Odometry odom;
            odom.header.stamp=now;
            odom.header.frame_id="odom";
            odom.child_frame_id="base_link";
            odom.pose.pose.position.x=this->x;
            odom.pose.pose.position.y=this->y;
            odom.pose.pose.position.z=0.0;
            odom.pose.pose.orientation.x=q.x();
            odom.pose.pose.orientation.y=q.y();
            odom.pose.pose.orientation.z=q.z();
            odom.pose.pose.orientation.w=q.w();
            odom.child_frame_id="base_link";
            odom.twist.twist.linear.x = this->vx;
            odom.twist.twist.linear.y = this->vy;
            odom.twist.twist.linear.z = 0.0;
            odom.twist.twist.angular.x=0.0;
            odom.twist.twist.angular.y=0.0;
            odom.twist.twist.angular.z=this->w;

            odom_publisher->publish(odom);

            vx=0.0;
            vy=0.0;
            w=0.0;
        }

        // Imu publisher
        void imu_pub_callback(){
            if (imu_data_available){
                rclcpp::Time now = this->get_clock()->now();
                sensor_msgs::msg::Imu imu;
                imu.header.stamp=now;
                imu.header.frame_id="imu_link";
                imu.linear_acceleration.x=ax+imu_offset_acc_x;
                imu.linear_acceleration.y=ay+imu_offset_acc_y;
                imu.linear_acceleration.z=az+imu_offset_acc_z;
                /*
                tf2::Quaternion q;
                q.setRPY(gx,gy,gz);
                imu.orientation.x=q.x();
                imu.orientation.y=q.y();
                imu.orientation.z=q.z();
                imu.orientation.w=q.w();
                */
                imu.orientation_covariance[0]=-1;
                imu.angular_velocity.x=gx+imu_offset_gyro_x;
                imu.angular_velocity.y=gy+imu_offset_gyro_y;
                imu.angular_velocity.z=gz+imu_offset_gyro_z;
                
                imu_publisher->publish(imu);
                imu_data_available = false;
            }
        }

        // Battery publisher
        void battery_pub_callback(){
            if (battery_data_available){
                rclcpp::Time now = this->get_clock()->now();
                sensor_msgs::msg::BatteryState battery_msg;
                battery_msg.header.stamp=now;
                battery_msg.header.frame_id="base_link";
                battery_msg.voltage=battery;
                battery_publisher->publish(battery_msg);
                battery_data_available=false;
            }
        }

        // This callback is used for dynamics parameters
        rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters){
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            for (const auto & param : parameters){

                if (param.get_name() == "publishTF"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    publishTF = param.as_bool();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }

                if (param.get_name() == "imu.offsets.accelerometer.x"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    imu_offset_acc_x = param.as_double();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                } 

                if (param.get_name() == "imu.offsets.accelerometer.y"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    imu_offset_acc_y = param.as_double();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }

                if (param.get_name() == "imu.offsets.accelerometer.z"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    imu_offset_acc_z = param.as_double();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }

                if (param.get_name() == "imu.offsets.gyro.x"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    imu_offset_gyro_x = param.as_double();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }

                if (param.get_name() == "imu.offsets.gyro.y"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    imu_offset_gyro_y = param.as_double();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }

                if (param.get_name() == "imu.offsets.gyro.z"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    imu_offset_gyro_z = param.as_double();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }
            }
            return result;
        }
        


        // Here are declared all parameters
        void parameters_declaration(){
            this->declare_parameter<std::string>("port_name","/dev/ttyUSB0");
            this->declare_parameter<float>("timeout_connection",60.0);

            this->declare_parameter<bool>("publishTF",true);

            this->declare_parameter<double>("imu.offsets.accelerometer.x",0.0);
            this->declare_parameter<double>("imu.offsets.accelerometer.y",0.0);
            this->declare_parameter<double>("imu.offsets.accelerometer.z",0.0);
            this->declare_parameter<double>("imu.offsets.gyro.x",0.0);
            this->declare_parameter<double>("imu.offsets.gyro.y",0.0);
            this->declare_parameter<double>("imu.offsets.gyro.z",0.0);

        }

        //Load static parameters
        void get_all_parameters(){
            this->get_parameter("port_name",device_name);
            this->get_parameter("timeout_connection",timeout_connection);

            this->get_parameter("publishTF",publishTF);

            this->get_parameter("imu.offsets.accelerometer.x",imu_offset_acc_x);
            this->get_parameter("imu.offsets.accelerometer.y",imu_offset_acc_y);
            this->get_parameter("imu.offsets.accelerometer.z",imu_offset_acc_z);
            this->get_parameter("imu.offsets.gyro.x",imu_offset_gyro_x);
            this->get_parameter("imu.offsets.gyro.y",imu_offset_gyro_y);
            this->get_parameter("imu.offsets.gyro.z",imu_offset_gyro_z);

        }


        void check_connection(){
            std::vector<uint8_t> serial_msg;
            uint8_t dim=packeter.packetC1F('E',timeout_connection);
            for(uint8_t i=0; i<dim; i++){
                serial_msg.push_back(packeter.msg[i]);
            }
            serial_driver->port()->async_send(serial_msg);
            serial_msg.clear();
        }





    public:

        AMR_Node():Node("AMR_node"),node_ctx{new IoContext(2)},serial_driver{new drivers::serial_driver::SerialDriver(*node_ctx)},packeter(100){
            vx=0.0;
            vy=0.0;
            w=0.0;
            x=0.0;
            y=0.0;
            theta=0.0;
            dt=0.0;
            battery=0.0;

            to_be_publish=false;
            imu_data_available=false;
            battery_data_available=false;
            
            connected = false;

            // Parameters
            parameters_declaration();
            get_all_parameters();
            parameters_callback_handle = add_on_set_parameters_callback(std::bind(&AMR_Node::parameters_callback, this, std::placeholders::_1));

            mecanum.setDimensions(0.0825,0.105,0.04);


            try{
                drivers::serial_driver::SerialPortConfig serial_config(115200,drivers::serial_driver::FlowControl::NONE,drivers::serial_driver::Parity::NONE,drivers::serial_driver::StopBits::ONE);
                serial_driver->init_port(device_name, serial_config); //modificare con device name

                previous_time=this->get_clock()->now();

                if (!serial_driver->port()->is_open()){
                    serial_driver->port()->open();
                    serial_driver->port()->async_receive(std::bind(&AMR_Node::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
                }
            }catch(const std::exception & e){
                RCLCPP_ERROR(get_logger(),"Error on creating serial port: %s",e.what());
            }

            tf_bc = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            joy_subscription = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",1,std::bind(&AMR_Node::joy_callback, this, std::placeholders::_1));
            initial_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>("/amr/initial_pose",1,std::bind(&AMR_Node::initial_pose_callback, this, std::placeholders::_1));
            
            odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
            odom_timer = this->create_wall_timer(10ms, std::bind(&AMR_Node::odom_pub_callback, this));

            imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/amr/imu/raw",1);
            imu_timer = this->create_wall_timer(10ms, std::bind(&AMR_Node::imu_pub_callback, this));

            battery_publisher = this->create_publisher<sensor_msgs::msg::BatteryState>("/amr/battery",1);
            battery_timer = this->create_wall_timer(1000ms, std::bind(&AMR_Node::battery_pub_callback, this));


            connection_timer = this->create_wall_timer(1000ms, std::bind(&AMR_Node::check_connection, this));


        }

        ~AMR_Node(){
            std::vector<uint8_t> serial_msg;
            uint8_t dim=packeter.packetC1F('S',0.0);
            for(uint8_t i=0; i<dim; i++){
                serial_msg.push_back(packeter.msg[i]);
            }
            serial_driver->port()->async_send(serial_msg);
            serial_msg.clear();


            if (node_ctx){
                node_ctx->waitForExit();
            }
        }


};



#endif
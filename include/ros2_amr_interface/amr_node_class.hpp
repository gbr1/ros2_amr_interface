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
        FIKmodel fik_model;
        std::string model;
        float model_lx, model_ly, model_wheel;
        double imu_offset_acc_x, imu_offset_acc_y, imu_offset_acc_z, imu_offset_gyro_x, imu_offset_gyro_y, imu_offset_gyro_z, acc_scale, gyro_scale;
        float vx, vy, w, x, y, theta, ax, ay, az, gx, gy, gz;
        double dt;
        float dtheta, dx, dy;
        float battery;
        bool publishTF;
        bool publishImu;
        bool publishBattery;
        float timeout_connection;
        bool connected;
        bool try_reconnect;


        bool to_be_publish;
        bool imu_data_available;
        bool battery_data_available;

        bool extra_verbose;

        std::string imu_link, odom_link, robot_link;

        // Required for serial communication
        std::unique_ptr<IoContext> node_ctx{};
        std::string device_name;
        int baud_rate;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver{};

        ucPack packeter;

      
        // Timers and times
        rclcpp::Time previous_time;
        rclcpp::TimerBase::SharedPtr odom_timer;
        rclcpp::TimerBase::SharedPtr imu_timer;
        rclcpp::TimerBase::SharedPtr battery_timer;
        rclcpp::TimerBase::SharedPtr connection_timer;
        rclcpp::Time timeout_time;


        rclcpp::TimerBase::SharedPtr send_timer;
        
        // Subscribers and Publishers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr initial_pose_subscription;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bc;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher;

        OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle;
        
        std::vector<uint8_t> serial_msg;
        uint8_t dim;
        bool cmd_is_exec;
        bool closing_is_exec;
        bool ask_to_close;


        // Callback for /cmd_vel topic subscription
        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            float w1,w2,w3,w4;
            fik_model.setVelocities(msg->linear.x,msg->linear.y,msg->angular.z);
            fik_model.forward();
            fik_model.getJoints(w1,w2,w3,w4);
            dim=packeter.packetC4F('J',w1,w2,w3,w4);
            for(uint8_t i=0; i<dim; i++){
                serial_msg.push_back(packeter.msg[i]);
            }
            cmd_is_exec=false;
            serial_driver->port()->async_send(serial_msg);
            send_timer = this->create_wall_timer(1ms, std::bind(&AMR_Node::send_callback, this));

            //--------------------------------------------------------------------
            if (extra_verbose){
                RCLCPP_INFO(this->get_logger(),"sent: %f\t%f\t%f\t%f",w1,w2,w3,w4);
            }
            //--------------------------------------------------------------------
        }

        // Callback to resend joint message until ack from hardware
        void send_callback(){
            if (!cmd_is_exec){
                serial_driver->port()->async_send(serial_msg);
            }
            else{
                serial_msg.clear();
                send_timer->cancel();
            }
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
            //---------------------------------------------------------------------------------------------
            if (extra_verbose){
                RCLCPP_INFO(this->get_logger(), "new pose:\t\t%f %f %f\t%f %f %f", x, y, 0.0, 0.0, 0.0, theta);
            }
            //---------------------------------------------------------------------------------------------
        }

        // Callback for serial communication
        void serial_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred){
            for (int i=0; i<int(bytes_transferred); i++){
                packeter.buffer.push(buffer[i]);
            }
            
            while (packeter.checkPayload()){
                uint8_t c=packeter.payloadTop();
                timeout_time=this->get_clock()->now();
                if (!connected){
                    if (c=='e'){
                        if (timeout_connection<=0.0){
                            connection_timer->cancel();
                        }
                        connected=true;
                        RCLCPP_INFO(this->get_logger(),"Hardware is now online");
                        //Set imu scales
                        setImuScales(acc_scale,gyro_scale);
                    }
                }
                else{
                    if (c=='j'){
                        float w1,w2,w3,w4;
                        packeter.unpacketC4F(c,w1,w2,w3,w4);
                        rclcpp::Time now = this->get_clock()->now();
                        //---------------------------------------------------------------------------
                        if (extra_verbose){
                            RCLCPP_INFO(this->get_logger(),"joints: %f\t%f\t%f\t%f",c,w1,w2,w3,w4);
                        }
                        //---------------------------------------------------------------------------
                        fik_model.setJoints(w1,w2,w3,w4);
                        fik_model.inverse();
                        fik_model.getVelocities(vx,vy,w);
                        dt=now.seconds()-previous_time.seconds();
                        previous_time=now;
                        
                        dtheta=w*dt;
                        dx=(vx*cos(theta)-vy*sin(theta))*dt;
                        dy=(vx*sin(theta)+vy*cos(theta))*dt;
                        x+=dx;
                        y+=dy;
                        theta+=dtheta;
                    } else
                    
                    // imu message from hardware
                    if ((c=='i')&&publishImu){
                        float temp, f;
                        packeter.unpacketC8F(c,ax,ay,az,gx,gy,gz,temp,f);
                        //-----------------------------------------------------------------------------------------
                        if (extra_verbose){
                            RCLCPP_INFO(this->get_logger(),"imu: %f\t%f\t%f\t%f\t%f\t%f\t%f",ax,ay,az,gx,gy,gz,temp);
                        }
                        //-----------------------------------------------------------------------------------------
                        imu_data_available=true;
                    } else
                
                    // battery message from hardware
                    if ((c=='b')&&publishBattery){
                        packeter.unpacketC1F(c,battery);
                        //-----------------------------------------------------------------------------------------
                        if (extra_verbose){
                            RCLCPP_INFO(this->get_logger(),"battery: %f V",battery);
                        }

                        //-----------------------------------------------------------------------------------------
                        battery_data_available=true;
                    } else

                    if (c=='s'){
                        closing_is_exec=true;
                        float f;
                        packeter.unpacketC1F(c,f);
                        //-----------------------------------------------------------------------------------------
                        RCLCPP_WARN(this->get_logger(),"Hardware is stopped");
                        //-----------------------------------------------------------------------------------------
                        if (try_reconnect&&!ask_to_close){
                            connected=false;
                            RCLCPP_WARN(this->get_logger(),"Try reconnecting to the hardware. If this message is repeated, check your hardware");
                        }
                        else{
                            rclcpp::shutdown();
                        }
                    } else

                    if ((c=='g')&&publishImu){
                        float a,g;
                        packeter.unpacketC2F(c,a,g);
                        if (a<=0){
                            RCLCPP_ERROR(this->get_logger(),"Wrong parameter on accelerometer scale: %f m/s^2", acc_scale);
                        }
                        if (g<=0){
                            RCLCPP_ERROR(this->get_logger(),"Wrong parameter on gyro scale: %f rad/s", gyro_scale);
                        }
                    } else

                    // hardware received joint command
                    if (c=='x'){
                        cmd_is_exec=true;
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
                t.header.frame_id = odom_link;
                t.child_frame_id = robot_link;

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
            odom.header.frame_id=odom_link;
            odom.child_frame_id=robot_link;
            odom.pose.pose.position.x=this->x;
            odom.pose.pose.position.y=this->y;
            odom.pose.pose.position.z=0.0;
            odom.pose.pose.orientation.x=q.x();
            odom.pose.pose.orientation.y=q.y();
            odom.pose.pose.orientation.z=q.z();
            odom.pose.pose.orientation.w=q.w();
            odom.child_frame_id=robot_link;
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
                imu.header.frame_id=imu_link;
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

                if (param.get_name() == "show_extra_verbose"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    extra_verbose = param.as_bool();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }

                if (param.get_name() == "try_reconnect"){
                    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
                    if (param.get_type() != correctType){
                        result.successful = false;
                        result.reason = param.get_name()+" setted as "+rclcpp::to_string(param.get_type())+" but declared as "+rclcpp::to_string(correctType);
                        RCLCPP_WARN_STREAM(get_logger(),result.reason);
                        return result;
                    }
                    try_reconnect = param.as_bool();
                    result.successful=true;
                    result.reason="Parameter "+param.get_name()+" setted correctly!";
                    return result;
                }

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


                if (publishImu&&(param.get_name() == "imu.offsets.accelerometer.x")){
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

                if (publishImu&&(param.get_name() == "imu.offsets.accelerometer.y")){
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

                if (publishImu&&(param.get_name() == "imu.offsets.accelerometer.z")){
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

                if (publishImu&&(param.get_name() == "imu.offsets.gyro.x")){
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

                if (publishImu&&(param.get_name() == "imu.offsets.gyro.y")){
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

                if (publishImu&&(param.get_name() == "imu.offsets.gyro.z")){
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
        
        void check_connection(){
            if (!connected){
                std::vector<uint8_t> serial_msg;
                uint8_t dim=packeter.packetC1F('E',timeout_connection);
                for(uint8_t i=0; i<dim; i++){
                    serial_msg.push_back(packeter.msg[i]);
                }
                serial_driver->port()->async_send(serial_msg);
                serial_msg.clear();
            }
            else{
                if ((this->get_clock()->now().seconds()-timeout_time.seconds())>timeout_connection){
                    RCLCPP_WARN(this->get_logger(),"serial connection is timed out, no message in about %f seconds despite %f setted", this->get_clock()->now().seconds()-timeout_time.seconds(),timeout_connection);
                    RCLCPP_WARN(this->get_logger(),"Trying to restart hardware");
                    connected=false;
                }
            }

        }

        // Here are declared all parameters
        void parameters_declaration(){
            this->declare_parameter<std::string>("port_name","/dev/ttyUSB0");
            this->declare_parameter<int>("baud_rate",115200);
            this->declare_parameter<float>("timeout_connection",60.0);
            this->declare_parameter<bool>("try_reconnect",true);
            this->declare_parameter<bool>("publishTF",true);

        
            this->declare_parameter<std::string>("odom.frame_id","odom");
            this->declare_parameter<std::string>("frame_id","base_link");

            this->declare_parameter<bool>("publishBattery",true);
            this->declare_parameter<bool>("show_extra_verbose", false);
        }

        // Load static parameters
        void get_all_parameters(){
            this->get_parameter("port_name",device_name);
            this->get_parameter("baud_rate",baud_rate);
            this->get_parameter("timeout_connection",timeout_connection);
            this->get_parameter("try_reconnect",try_reconnect);
            this->get_parameter("publishTF",publishTF);

            this->get_parameter("odom.frame_id",odom_link);
            this->get_parameter("frame_id",robot_link);   

            this->get_parameter("publishBattery", publishBattery);
            this->get_parameter("show_extra_verbose", extra_verbose);    

        }

        // Math model parameters
        void model_parameters(){

            this->declare_parameter<std::string>("model.type","mecanum");
            this->get_parameter("model.type",model);

            if (model.compare("mecanum")==0){
                this->declare_parameter<float>("model.size.chassis.x",0.0825);
                this->declare_parameter<float>("model.size.chassis.y",0.105);
                this->declare_parameter<float>("model.size.wheel.radius",0.04);

                this->get_parameter("model.size.chassis.x",model_lx);
                this->get_parameter("model.size.chassis.y",model_ly);
                this->get_parameter("model.size.wheel.radius",model_wheel);

                fik_model.setModel(FIKmodel::model::MECANUM);
                fik_model.setDimensions(model_lx, model_ly, model_wheel);
            } else

            if (model.compare("differential")==0){
                this->declare_parameter<float>("model.size.chassis.wheel_separation",0.15);
                this->declare_parameter<float>("model.size.wheel.radius",0.0325);

                fik_model.setModel(FIKmodel::model::DIFFERENTIAL);
                this->get_parameter("model.size.chassis.wheel_separation",model_ly);
                this->get_parameter("model.size.wheel.radius",model_wheel);
                fik_model.setDimensions(model_ly, model_wheel);
            } else

            if (model.compare("skid")==0){
                this->declare_parameter<float>("model.size.chassis.wheel_separation",0.0825);
                this->declare_parameter<float>("model.size.wheel.radius",0.105);

                fik_model.setModel(FIKmodel::model::SKID);
                this->get_parameter("model.size.chassis.wheel_separation",model_ly);
                this->get_parameter("model.size.wheel.radius",model_wheel);
                fik_model.setDimensions(model_ly, model_wheel);
            } else {
                RCLCPP_ERROR(this->get_logger(),"wrong paramenter on model.type, it can't be %s", model.c_str());
                this->~AMR_Node();
            }
            
                       
        }

        // IMU parameters
        void imu_parameters(){
            this->declare_parameter<bool>("publishIMU",true);
            this->get_parameter("publishIMU", publishImu);

            if (publishImu){
                this->declare_parameter<std::string>("imu.frame_id","imu_link");
                this->declare_parameter<double>("imu.offsets.accelerometer.x",0.0);
                this->declare_parameter<double>("imu.offsets.accelerometer.y",0.0);
                this->declare_parameter<double>("imu.offsets.accelerometer.z",0.0);
                this->declare_parameter<double>("imu.offsets.gyro.x",0.0);
                this->declare_parameter<double>("imu.offsets.gyro.y",0.0);
                this->declare_parameter<double>("imu.offsets.gyro.z",0.0);
                this->declare_parameter<float>("imu.scale.accelerometer",2.0);
                this->declare_parameter<float>("imu.scale.gyro",250.0);
            
                this->get_parameter("imu.frame_id",imu_link);
                this->get_parameter("imu.offsets.accelerometer.x",imu_offset_acc_x);
                this->get_parameter("imu.offsets.accelerometer.y",imu_offset_acc_y);
                this->get_parameter("imu.offsets.accelerometer.z",imu_offset_acc_z);
                this->get_parameter("imu.offsets.gyro.x",imu_offset_gyro_x);
                this->get_parameter("imu.offsets.gyro.y",imu_offset_gyro_y);
                this->get_parameter("imu.offsets.gyro.z",imu_offset_gyro_z);
                this->get_parameter("imu.scale.accelerometer",acc_scale);
                this->get_parameter("imu.scale.gyro",gyro_scale);
            }
        }

        void setImuScales(const float acc, const float gyro){
            std::vector<uint8_t> serial_msg;
            uint8_t dim=packeter.packetC2F('G',acc,gyro);
            for(uint8_t i=0; i<dim; i++){
                serial_msg.push_back(packeter.msg[i]);
            }
            serial_driver->port()->async_send(serial_msg);
            serial_msg.clear();
        }

    public:

        AMR_Node():Node("AMR_node"),node_ctx(new IoContext(2)),serial_driver(new drivers::serial_driver::SerialDriver(*node_ctx)),packeter(100){
            vx=0.0;
            vy=0.0;
            w=0.0;
            x=0.0;
            y=0.0;
            theta=0.0;
            dt=0.0;
            dtheta=0.0;
            dx=0.0;
            dy=0.0;
            battery=0.0;

            to_be_publish=false;
            imu_data_available=false;
            battery_data_available=false;
            
            connected = false;

            cmd_is_exec=false;
            closing_is_exec=false;
            ask_to_close=false;

            // Parameters
            parameters_declaration();
            get_all_parameters();
            model_parameters();
            imu_parameters();
            parameters_callback_handle = add_on_set_parameters_callback(std::bind(&AMR_Node::parameters_callback, this, std::placeholders::_1));
            


            try{
                drivers::serial_driver::SerialPortConfig serial_config(baud_rate,drivers::serial_driver::FlowControl::NONE,drivers::serial_driver::Parity::NONE,drivers::serial_driver::StopBits::ONE);
                serial_driver->init_port(device_name, serial_config);

                previous_time=this->get_clock()->now();

                if (!serial_driver->port()->is_open()){
                    serial_driver->port()->open();
                }

                RCLCPP_INFO(get_logger(),"Serial opened on %s at 115200",device_name.c_str());


                //ROS2 stuffs
                timeout_time=this->get_clock()->now();

                tf_bc = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                cmd_subscription = this->create_subscription<geometry_msgs::msg::Twist>("/amr/cmd_vel",1,std::bind(&AMR_Node::cmd_callback, this, std::placeholders::_1));
                initial_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>("/amr/initial_pose",1,std::bind(&AMR_Node::initial_pose_callback, this, std::placeholders::_1));
                
                odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/amr/odometry",1);
                odom_timer = this->create_wall_timer(10ms, std::bind(&AMR_Node::odom_pub_callback, this));
                
                if (publishImu){
                    imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/amr/imu/raw",1);
                    imu_timer = this->create_wall_timer(10ms, std::bind(&AMR_Node::imu_pub_callback, this));
                }

                if (publishBattery){
                    battery_publisher = this->create_publisher<sensor_msgs::msg::BatteryState>("/amr/battery",1);
                    battery_timer = this->create_wall_timer(1000ms, std::bind(&AMR_Node::battery_pub_callback, this));
                }

                if (serial_driver->port()->is_open()){
                    serial_driver->port()->async_receive(std::bind(&AMR_Node::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
                    connection_timer = this->create_wall_timer(1000ms, std::bind(&AMR_Node::check_connection, this));
                }

            }catch(const std::exception & e){
                RCLCPP_ERROR(get_logger(),"Error on creating serial port: %s",device_name.c_str());
                this->~AMR_Node();
            }
        }

        ~AMR_Node(){
            closing_is_exec=false;
            ask_to_close=true;
            if (serial_driver->port()->is_open()){
                std::vector<uint8_t> serial_msg;
                uint8_t dim=packeter.packetC1F('S',0.0);
                for(uint8_t i=0; i<dim; i++){
                    serial_msg.push_back(packeter.msg[i]);
                }
                closing_is_exec=false;
                while(!closing_is_exec){
                    serial_driver->port()->async_send(serial_msg);
                    usleep(10000);
                }
                serial_msg.clear();
            }
            

            if (node_ctx){
                node_ctx->waitForExit();
            }
        }


};



#endif
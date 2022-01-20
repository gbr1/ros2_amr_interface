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

#include "ros2_amr_interface/FIKmodel.hpp"
#include "ucPack/ucPack.h"


std::string port_name="/dev/ttyUSB0";

FIKmodel mecanum(0.0825,0.105,0.04); //lx=0.0825m, ly=0.105m , r= 0.04m
//FIKmodel mecanum(0.165,0.21,0.04);
ucPack packeter(100);

class AMR_Node: public rclcpp::Node{
  private:
      //serial
      std::unique_ptr<IoContext> node_ctx{};
      std::string device_name{};
      std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config;
      std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver;
      
      rclcpp::Time previous_time;


      
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_subscription;






      void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
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


      //receive wheels speed
      void serial_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred){
        for (int i=0; i<int(bytes_transferred); i++){
          packeter.buffer.push(buffer[i]);
          //std::cout<<std::hex<<int(buffer[i])<<std::dec<<" ";
        }
        std::cout<<std::endl;
        
        while (packeter.checkPayload()){
          uint8_t c;
          float w1,w2,w3,w4;
          packeter.unpacketC4F(c,w1,w2,w3,w4);
          rclcpp::Time now = this->get_clock()->now();
          RCLCPP_INFO(this->get_logger(),"joints: %f\t%f\t%f\t%f",w1,w2,w3,w4);
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
          
          to_be_publish=true;
        }
      }





  public:
    float vx, vy, w, x, y, theta;
    double dt;
    bool to_be_publish;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bc;



    AMR_Node():Node("AMR_node"),node_ctx{new IoContext(2)},serial_driver{new drivers::serial_driver::SerialDriver(*node_ctx)}{
      vx=0;
      vy=0;
      w=0;
      x=0;
      y=0;
      theta=0;
      dt=0;
      to_be_publish=false;

      try{
        
        drivers::serial_driver::SerialPortConfig serial_config(115200,drivers::serial_driver::FlowControl::NONE,drivers::serial_driver::Parity::NONE,drivers::serial_driver::StopBits::ONE);
        serial_driver->init_port(port_name, serial_config); //modificare con device name

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
    }

    ~AMR_Node(){
      if (node_ctx){
        node_ctx->waitForExit();
      }
    }


};






int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<AMR_Node>());


  auto amr_node = std::make_shared<AMR_Node>();
  auto odom_publisher = amr_node->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",rclcpp::QoS{1});
  rclcpp::Rate loop_rate(100); //50hz



  while(rclcpp::ok()){
    if (amr_node->to_be_publish){

      rclcpp::Time now = amr_node->get_clock()->now();
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = now;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";

      t.transform.translation.x = amr_node->x;
      t.transform.translation.y = amr_node->y;
      t.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0,0.0,amr_node->theta);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      amr_node->tf_bc->sendTransform(t);
      



      nav_msgs::msg::Odometry odom;
      odom.header.stamp=now;
      odom.header.frame_id="odom";
      odom.child_frame_id="base_link";
      odom.pose.pose.position.x=amr_node->x;
      odom.pose.pose.position.y=amr_node->y;
      odom.pose.pose.position.z=0.0;
      odom.pose.pose.orientation.x=q.x();
      odom.pose.pose.orientation.y=q.y();
      odom.pose.pose.orientation.z=q.z();
      odom.pose.pose.orientation.w=q.w();
      odom.child_frame_id="base_link";
      odom.twist.twist.linear.x = amr_node->vx;
      odom.twist.twist.linear.y = amr_node->vy;
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x=0.0;
      odom.twist.twist.angular.y=0.0;
      odom.twist.twist.angular.z=amr_node->w;

      odom_publisher->publish(odom);
      amr_node->to_be_publish = false;
    }
    rclcpp::spin_some(amr_node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
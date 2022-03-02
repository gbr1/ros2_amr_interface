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

#ifndef __FIKmodel_HPP__
#define __FIKmodel_HPP__

#include <stdint.h>
#include <vector>

class FIKmodel{
    private:
        float lx,ly,wheel_radius;
        float vx, vy, w;
        std::vector<float> joints;

    public:
        std::function<void()> forward;
        std::function<void()> inverse;
        enum model { MECANUM, DIFFERENTIAL, SKID};

        FIKmodel(){
            //put something wrong and different by 0.0
            this->lx=-1.0;
            this->ly=-1.0;
            this->wheel_radius=-1.0;
            vx=0.0;
            vy=0.0;
            w=0.0;
        };

        FIKmodel(const model m){
            setModel(m);
            vx=0.0;
            vy=0.0;
            w=0.0;
            for (uint8_t i=0; i<joints.size(); i++){
                joints[i]=0.0;
            }
        }
    
        void setModel(const model m=MECANUM){
            if (m==MECANUM){
                forward=std::bind(&FIKmodel::forward_mecanum,this);
                inverse=std::bind(&FIKmodel::inverse_mecanum,this);
                joints.resize(4);
            } else
            if (m==DIFFERENTIAL){
                forward=std::bind(&FIKmodel::forward_differential,this);
                inverse=std::bind(&FIKmodel::inverse_differential,this);
                joints.resize(4);
            } else
            if (m==SKID){
                forward=std::bind(&FIKmodel::forward_skid,this);
                inverse=std::bind(&FIKmodel::inverse_skid,this);
                joints.resize(4);
            }
        }
    
    
        //------------------------------------------------------------------------------------//
        //                                  MECANUM MODEL                                     //
        //------------------------------------------------------------------------------------//
    
        /*
         
                      ly
                  <------->
         
                +---+           +---+
                | 0 |-----------| 1 |       ^
                +---+     ^     +---+       |
                  |   \   ^   /   |         | lx
                  |       ^       |         |
                  |       O       |         v
                  |               |
                  |   /       \   |
                +---+           +---+
                | 2 |-----------| 3 |
                +---+           +---+
         
                0: left_front_joint
                1: right_front_joint
                2: left_rear_joint
                3: right_rear_joint
         
        */
        
        void forward_mecanum(){
            joints[0]=(vx-vy-w*(lx+ly))/wheel_radius;
            joints[1]=(vx+vy+w*(lx+ly))/wheel_radius;
            joints[2]=(vx+vy-w*(lx+ly))/wheel_radius;
            joints[3]=(vx-vy+w*(lx+ly))/wheel_radius;
        }
    
        void inverse_mecanum(){
            vx=(joints[0]+joints[1]+joints[2]+joints[3])*wheel_radius/4.0;
            vy=(-joints[0]+joints[1]+joints[2]-joints[3])*wheel_radius/4.0;
            w=(-joints[0]+joints[1]-joints[2]+joints[3])*wheel_radius/(4.0*(lx+ly));
        }
    
        //------------------------------------------------------------------------------------//
        //                                  DIFFERENTIAL MODEL                                //
        //------------------------------------------------------------------------------------//

        /*
         
                         ly
                  <--------------->
                    
                  +---------------+
                  |       ^       |
                  |       ^       |         
                +---+     ^     +---+
                |   |           |   |
                | 0 |     O     | 1 |
                |   |           |   |
                +---+           +---+
                  |               |
                  |               |
                  +---------------+
         
                0: left_joint
                1: right_joint
         
        */
    
        void forward_differential(){
            joints[0]=(2.0*vx-w*ly)/(2.0*wheel_radius);
            joints[1]=(2.0*vx+w*ly)/(2.0*wheel_radius);
            joints[2]=0.0;
            joints[3]=0.0;
        }

        void inverse_differential(){
            vx=(joints[0]+joints[1])*wheel_radius/2.0;
            vy=0.0;
            w=(-joints[0]+joints[1])*wheel_radius/ly;
        }

    
    
        //------------------------------------------------------------------------------------//
        //                                  SKID MODEL                                        //
        //------------------------------------------------------------------------------------//

        /*
         
                      ly
                  <------->
         
                +---+           +---+
                | 0 |-----------| 1 |       ^
                +---+     ^     +---+       |
                  |       ^       |         | lx
                  |       ^       |         |
                  |       O       |         v
                  |               |
                  |               |
                +---+           +---+
                | 2 |-----------| 3 |
                +---+           +---+
         
                0: left_front_joint
                1: right_front_joint
                2: left_rear_joint
                3: right_rear_joint
         
        */

        void forward_skid(){
            joints[0]=(vx-ly*w)/wheel_radius;
            joints[1]=(vx+ly*w)/wheel_radius;
            joints[2]=(vx-ly*w)/wheel_radius;
            joints[3]=(vx+ly*w)/wheel_radius;
        }

        void inverse_skid(){
            vx=wheel_radius*(joints[0]+joints[1]+joints[2]+joints[3])/4.0;
            vy=0.0; //no drift case
            w=wheel_radius*(-joints[0]+joints[1]-joints[2]+joints[3])/(4.0*ly);
        }
    
    
    
    
    
    
    
        //------------------------------------------------------------------------------------//
        //                                  Generic interface                                 //
        //------------------------------------------------------------------------------------//
    
        void setDimensions(const float ly, const float r){
            this->lx=0.0; //unused for differential and skid if you don't want to take care about drifting
            this->ly=ly;
            this->wheel_radius=r;
        }
        
        void setDimensions(const float lx, const float ly, const float r){
            this->lx=lx;
            this->ly=ly;
            this->wheel_radius=r;
        }
    
        void getJoints(float & w0, float & w1){
            w0=joints[0];
            w1=joints[1];
        }
        
        void getJoints(float & w0, float & w1, float & w2, float & w3){
            w0=joints[0];
            w1=joints[1];
            w2=joints[2];
            w3=joints[3];
        }
    
        void setVelocities(const float vel_x, const float vel_w){
            vx=vel_x;
            w=vel_w;
        }
    
        void setVelocities(const float vel_x, const float vel_y, const float vel_w){
            vx=vel_x;
            vy=vel_y;
            w=vel_w;
        }

        void setJoints(const float w0, const float & w1){
            joints[0]=w0;
            joints[1]=w1;
        }
        
        void setJoints(const float w0, const float w1, const float w2, const float w3){
            joints[0]=w0;
            joints[1]=w1;
            joints[2]=w2;
            joints[3]=w3;
        }
    
        void getVelocities(float & vel_x, float & vel_w){
            vel_x=vx;
            vel_w=w;
        }
    
        void getVelocities(float & vel_x, float & vel_y, float & vel_w){
            vel_x=vx;
            vel_y=vy;
            vel_w=w;
        }
        
};


#endif /* FIKmodel_h */

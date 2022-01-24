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

#include "ucPack.h"

ucPack::ucPack(const uint8_t buffer_size, const uint8_t start_index, const uint8_t end_index):buffer(buffer_size){
    this->start_index=start_index;
    this->end_index=end_index;
    payload=new uint8_t[buffer_size];
    payload_size = 0;
    msg = new uint8_t[buffer_size];
    msg_size = 0;
}

ucPack::~ucPack(){
    delete [] msg;
    delete [] payload;
}

bool ucPack::checkPayload(){
    //check if buffer is empty
    if (buffer.isEmpty()){
        return false;
    }
    
    //check the index byte
    while ((buffer.top()!=start_index)&&(buffer.getSize()>0)){
        buffer.pop(); //discard the first byte
    }
    
    //exit if only message index is received
    if (buffer.getSize()<=1){
        return false;
    }
    
    //get the payload dimension
    payload_size=buffer[1];

    //check if the packet is complete
    if (buffer.getSize()<(4+payload_size)){  //memo: index|length|msg|stop|crc8
        return false;
    }
    
    //check if stop byte is correct
    if (buffer[2+payload_size]!=end_index){
        return false;
    }
    
    //crc checking
    //memcpy(payload,buffer.ptr()+2,payload_size);
    for(uint8_t i=0; i<payload_size;i++){
        payload[i]=buffer[i+2];
    }
    
    if (crc8(payload,payload_size)!=buffer[3+payload_size]){
        buffer.pop(); //delete the index, so it is possible to recheck
        return false;
    }
    

    //clear the buffer
    for (uint8_t i=0; i<4+payload_size; i++){     
        buffer.pop();
    }
    payload_size=0;
    
    return true;
}

/*
void ucPack::show(){
    for (uint8_t i=0; i<payload_size; i++) {
        std::cout<<int(i)<<":\t"<<std::hex<<int(payload[i])<<std::dec<<std::endl;
    }
}*/

uint8_t ucPack::crc8(const uint8_t *data, const uint8_t size){
    uint8_t crc = 0x00;
    uint8_t extract;
    uint8_t sum;
    for(uint8_t i=0;i<size;i++){
        extract = *data;
        for (uint8_t j = 8; j>0; j--){
            sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum){
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
        data++;
    }
    return crc;
}

uint8_t ucPack::payloadTop(){
    return payload[0];
}

/*
uint8_t ucPack::packetize(const char *types, const uint8_t n, ...){
    msg[0]=start_index;
    va_list vl;
    uint8_t msg_size=2;
    for (uint8_t i=0; i<n; i++){
        switch (types[i]){
            case 'c':
                msg[msg_size]=uint8_t(va_arg(vl,char));
                msg_size++;
                break;
            case 'f':
                float f = float(va_arg(vl,float));
                memcpy(msg+msg_size, &f, sizeof(float));
                msg_size+=sizeof(float);
                break;
        }
    }
    msg[msg_size]=end_index;
    msg[1]=msg_size-4;
    return msg_size;
}
 */

uint8_t ucPack::packetC1F(const uint8_t code, const float f){
    msg[0]=start_index;
    msg[1]=5;
    msg[2]=code;
    memcpy(msg+3,&f,sizeof(float));
    msg[7]=end_index;
    msg[8]=crc8(msg+2,5);
    msg_size=9;
    return msg_size;
}

void ucPack::unpacketC1F(uint8_t &code, float &f){
    code=payload[0];
    memcpy(&f, payload+1, sizeof(float));
}

uint8_t ucPack::packetC4F(const uint8_t code, const float f1, const float f2, const float f3, const float f4){
    msg[0]=start_index;
    msg[1]=17;
    msg[2]=code;
    memcpy(msg+3,&f1,sizeof(float));
    memcpy(msg+7,&f2,sizeof(float));
    memcpy(msg+11,&f3,sizeof(float));
    memcpy(msg+15,&f4,sizeof(float));
    msg[19]=end_index;
    msg[20]=crc8(msg+2,17);
    msg_size=21;
    return msg_size;
}

void ucPack::unpacketC4F(uint8_t &code, float &f1, float &f2, float &f3, float &f4){
    code=payload[0];
    memcpy(&f1, payload+1, sizeof(float));
    memcpy(&f2, payload+5, sizeof(float));
    memcpy(&f3, payload+9, sizeof(float));
    memcpy(&f4, payload+13, sizeof(float));
}

uint8_t ucPack::packetC8F(const uint8_t code,const float f1, const float f2, const float f3, const float f4,
                  const float f5, const float f6, const float f7, const float f8){
    msg[0]=start_index;
    msg[1]=33;
    msg[2]=code;
    memcpy(msg+3,&f1,sizeof(float));
    memcpy(msg+7,&f2,sizeof(float));
    memcpy(msg+11,&f3,sizeof(float));
    memcpy(msg+15,&f4,sizeof(float));
    memcpy(msg+19,&f5,sizeof(float));
    memcpy(msg+23,&f6,sizeof(float));
    memcpy(msg+27,&f7,sizeof(float));
    memcpy(msg+31,&f8,sizeof(float));
    msg[35]=end_index;
    msg[36]=crc8(msg+2,33);
    msg_size=37;
    return msg_size;
}

void ucPack::unpacketC8F(uint8_t &code, float &f1, float &f2, float &f3, float &f4,
                 float &f5, float &f6, float &f7, float &f8){
    code=payload[0];
    memcpy(&f1, payload+1, sizeof(float));
    memcpy(&f2, payload+5, sizeof(float));
    memcpy(&f3, payload+9, sizeof(float));
    memcpy(&f4, payload+13, sizeof(float));
    memcpy(&f5, payload+17, sizeof(float));
    memcpy(&f6, payload+21, sizeof(float));
    memcpy(&f7, payload+25, sizeof(float));
    memcpy(&f8, payload+29, sizeof(float));
}





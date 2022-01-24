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

#ifndef ucPack_hpp
#define ucPack_hpp

#include "CircularBuffer.h"
#include <stdint.h>
#include <string.h>
//#include <stdarg.h>

class ucPack {
    private:
        uint8_t start_index;
        uint8_t end_index;
        uint8_t * payload;
        uint8_t payload_size;
        uint8_t msg_size;
    public:
        uint8_t * msg;
        CircularBuffer buffer;
        ucPack(const uint8_t buffer_size, const uint8_t start_index='A', const uint8_t end_index='#');
    
        bool checkPayload();
        //void show();
    
        uint8_t crc8(const uint8_t * data,const uint8_t size);
        uint8_t payloadTop();
        //uint8_t packetize(const char * types, const uint8_t n, ...); //put bytes in msg and return msg_size;
        
        uint8_t packetC1F(const uint8_t code,const float f);
        void unpacketC1F(uint8_t &code, float &f);
    
            
        uint8_t packetC4F(const uint8_t code,const float f1, const float f2, const float f3, const float f4);
        void unpacketC4F(uint8_t &code, float &f1, float &f2, float &f3, float &f4);
    
        uint8_t packetC8F(const uint8_t code,const float f1, const float f2, const float f3, const float f4,
                                             const float f5, const float f6, const float f7, const float f8);
        void unpacketC8F(uint8_t &code, float &f1, float &f2, float &f3, float &f4,
                                        float &f5, float &f6, float &f7, float &f8);
    
        ~ucPack();
};

#endif /* ucPack_hpp */

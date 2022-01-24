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

#ifndef CircularBuffer_hpp
#define CircularBuffer_hpp

#include <stdint.h>

class CircularBuffer {
    private:
        uint8_t * buffer;
        uint8_t head;
        uint8_t tail;
        uint8_t capacity;
        uint8_t size;
    public:
        CircularBuffer(const uint8_t dimension);
        ~CircularBuffer();

        bool isEmpty();
        bool isFull();
        
        void push(const uint8_t element);
        uint8_t pop();
        uint8_t top();

        //void show();
    
        void insert(const uint8_t * to_be_copied, const uint8_t dimension);
    
        uint8_t getSize();
    
        //void copy(uint8_t * copied, uint8_t & dimension);
    
        uint8_t& operator[](int);
    
        uint8_t * ptr();
    
};


#endif /* CircularBuffer_hpp */

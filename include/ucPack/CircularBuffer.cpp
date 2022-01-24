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

#include "CircularBuffer.h"


CircularBuffer::CircularBuffer(const uint8_t dimension){
    capacity=dimension;
    buffer = new uint8_t[dimension];
    head=0;
    tail=0;
    size=0;
}

CircularBuffer::~CircularBuffer(){
    delete buffer;
}

void CircularBuffer::push(const uint8_t element){
    buffer[tail]=element;
    if (!isFull()){
        size++;
    }
    tail=(tail+1)%capacity;
    if (size==capacity){
        head=tail;
    }
    //std::cout<<"PUSH: "<<element<<"\t head: "<<head<<"\t tail: "<<tail<<"\t size: "<<size<<"\r\n";
}

uint8_t CircularBuffer::pop(){
    if (isEmpty()){
        return 0;
    }
    uint8_t element=buffer[head];
    head=(head+1)%capacity;
    size--;
    //std::cout<<"POP: "<<element<<"\t head: "<<head<<"\t tail: "<<tail<<"\t size: "<<size<<"\r\n";
    return element;
}

uint8_t CircularBuffer::top(){
    if (isEmpty()){
        return 0;
    }
    return buffer[head];
}

/*
void CircularBuffer::show(){
    if(!isEmpty()){
        uint8_t index = head;
        for (uint8_t i=0; i<size; i++){
            std::cout<<int(index)<<"\t:\t"<<buffer[index]<<"\t:\t"<<int(buffer[index])<<std::endl;
            index=(index+1)%capacity;
        }
    }
}
 */


bool CircularBuffer::isEmpty(){
    return (size==0);
}

bool CircularBuffer::isFull(){
    return (size==capacity);
}

void CircularBuffer::insert(const uint8_t * to_be_copied, const uint8_t dimension){
    uint8_t copy_index = 0;
    if (capacity<dimension){
        copy_index=dimension-capacity;
    }
    for(uint8_t i=copy_index; i<dimension; i++){
        push(to_be_copied[i]);
    }
}

uint8_t CircularBuffer::getSize(){
    return size;
}

uint8_t& CircularBuffer::operator[](int index){
    return buffer[(head+index)%capacity];
}

uint8_t * CircularBuffer::ptr(){
    return buffer;
}


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


#ifndef _FIK_MODEL_HPP_
#define _FIK_MODEL_HPP

class FIKmodel{
    private:
        float lx,ly,r;
    public:
        FIKmodel(){
            this->lx=1.0;
            this->ly=1.0;
            this->r=1.0;
        };

        FIKmodel(const float lx, const float ly, const float r){
            this->lx=lx;
            this->ly=ly;
            this->r=r;
        }

        void forward(const float vx, const float vy, const float w, float & w1, float & w2, float & w3, float & w4){
            w1=(vx-vy-w*(lx+ly))/r;
            w2=(vx+vy+w*(lx+ly))/r;
            w3=(vx+vy-w*(lx+ly))/r;
            w4=(vx-vy+w*(lx+ly))/r;
        }

        void inverse(const float w1, const float w2, const float w3, const float w4, float & vx, float & vy, float & w){
            vx=(w1+w2+w3+w4)*r/4.0;
            vy=(-w1+w2+w3-w4)*r/4.0;
            w=(-w1+w2-w3+w4)*r/(4.0*(lx+ly));
        }

        void setDimensions(const float lx, const float ly, const float r){
            this->lx=lx;
            this->ly=ly;
            this->r=r;
        }
};

#endif
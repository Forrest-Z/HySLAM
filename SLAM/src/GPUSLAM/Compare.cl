    /**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/
__kernel void Compare(__global float *inputbuffer,__global float *input2buffer,__global float *outputbuffer)
{
	float inputMainMap;
    float inputMiniMap;

    uint global_addr;
    global_addr = get_global_id(0);

    
	inputMainMap = inputbuffer[global_addr];
    inputMiniMap = input2buffer[global_addr];

    if(inputMainMap < 0.8 && inputMainMap > -0.8){
        outputbuffer[global_addr] =  99.0 ;
    }
    else if(inputMiniMap < 0.01 && inputMiniMap > -0.01){
        outputbuffer[global_addr] = 99.0 ;
    }else if (inputMiniMap < 0.00 && inputMainMap < 0.0){
        outputbuffer[global_addr] =  99.0 ;
    }
 
    else{
        float diff = 0.0;
        diff = inputMainMap - inputMiniMap;
        if(diff < 0.0){
            diff = diff * -1.0;
        }


        outputbuffer[global_addr] = diff ;
    }
    

	
}
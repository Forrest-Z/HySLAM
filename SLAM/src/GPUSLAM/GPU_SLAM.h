//class manages GPU for GPUGM-SLAM

#pragma once

#include "../Map/Map.h"
#include "../ImportData/Constants.h"
#include "MiniMap.h"
#include "../Robot/Robot.h"
#include "../Interface.h"
#include "../Data.h"
#include "../Log/Logging.h"

#include <CL/cl.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>

#include <chrono> //time

using namespace std::chrono;


#define SUCCESS 0
#define FAILURE 1
#define EXPECTED_FAILURE 2

//#define  GlobalThreadSize 1000
const cl_uint GlobalThreadSize = 50000;//treads used
//could make this based on harware - clGetKernelWorkGroupInfo
#define  GroupSize 128


class GPU_SLAM
{
	Map* map;
	Data* data;
	Robot* robot;

	std::vector<float> test1;
	std::vector<float> test2;

	cl_int status = 0;//store the return status

	cl_device_id device;
	cl_program program;
	//cl_program program2;
	cl_context context;

	cl_kernel kernel;
	//cl_kernel kernel2;
	cl_command_queue commandQueue;

	//set input data
	cl_uint inputSizeBytes = GlobalThreadSize * sizeof(cl_uint);
	//cl_float* input = (cl_float*)malloc(inputSizeBytes * (cl_uint)4);
	//cl_float* input2 = (cl_float*)malloc(inputSizeBytes * (cl_uint)4);

	cl_float* input = (cl_float*)malloc(inputSizeBytes);
	cl_float* input2 = (cl_float*)malloc(inputSizeBytes);

	//cl_float* input3 = (cl_float*)malloc(363 * sizeof(cl_uint) );

	float output[GlobalThreadSize];

	cl_mem outputBuffer;
	cl_mem input2Buffer;
	cl_mem inputBuffer;
	//cl_mem input3Buffer;

	size_t global_threads;
	size_t local_threads;

public: 
	GPU_SLAM();
	void setUp(Map* newmap, Data* newdata, Robot* newrobot);
	void cleanUp();
	float getScore(Offset offset);

private:
	int convertToString(const char* filename, std::string& s);
	cl_device_id create_device();
	cl_program build_program(cl_context ctx, cl_device_id dev, const char* filename);
	void createInput(std::vector < MMTile> miniMap);
	float runGPU();
	//float runGPUMapCreation();
	//void createInputForMap();
	
};


/*
	HySLAM: SLAM algorithm
	Copyright (C) 2020  Samuel Haley

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	Contact: SH275@hw.ac.uk
*/


#include "GPU_SLAM.h"


GPU_SLAM::GPU_SLAM() {

}

 void GPU_SLAM::setUp(Map* newmap, Data* newdata, Robot* newrobot) {

	 map = newmap;
	 data = newdata;
	 robot = newrobot;
	// size_t global_threads;
	// size_t local_threads;
	 global_threads = GlobalThreadSize;
	 //local_threads = GroupSize;

	//create device
	device = create_device();
	/* Create Context using the platform selected above */
	context = clCreateContext(NULL, 1, &device, NULL, NULL, &status);
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Creating context failed!" << std::endl;
		exit(1);
	}

	program = build_program(context, device, "src/GPUSLAM/Compare.cl");
	//program2 = build_program(context, device, "src/GPUSLAM/CreateMap.cl");

	/* Create data buffer

   • `global_size`: total number of work items that will be
	  executed on the GPU (e.g. total size of your array)
   • `local_size`: size of local workgroup. Each workgroup contains
	  several work items and goes to a compute unit

   In this example, the kernel is executed by eight work-items divided into
   two work-groups of four work-items each. Returning to my analogy,
   this corresponds to a school containing eight students divided into
   two classrooms of four students each.

	 Notes:
   • Intel recommends workgroup size of 64-128. Often 128 is minimum to
   get good performance on GPU
   • On NVIDIA Fermi, workgroup size must be at least 192 for full
   utilization of cores
   • Optimal workgroup size differs across applications
   */
	//create input buffer
	inputBuffer = clCreateBuffer(
		context,
		CL_MEM_READ_ONLY |
		CL_MEM_USE_HOST_PTR,
		sizeof(cl_uint) * GlobalThreadSize ,
		input,
		&status);
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Creating input buffer failed!" << std::endl;
		exit(1);
	}

	input2Buffer = clCreateBuffer(
		context,
		CL_MEM_READ_ONLY |
		CL_MEM_USE_HOST_PTR,
		sizeof(cl_uint) * GlobalThreadSize ,
		input2,
		&status);
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Creating input buffer failed!" << std::endl;
		exit(1);
	}

	/*input3Buffer = clCreateBuffer(
		context,
		CL_MEM_READ_ONLY |
		CL_MEM_USE_HOST_PTR,
		sizeof(cl_uint) * 363,
		input3,
		&status);
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Creating input buffer failed!" << std::endl;
		exit(1);
	}*/


	//create output buffer
	outputBuffer = clCreateBuffer(
		context,
		CL_MEM_READ_WRITE |
		CL_MEM_COPY_HOST_PTR,
		sizeof(cl_uint) * GlobalThreadSize,
		output,
		&status);
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Creating output buffer failed!" << std::endl;
		exit(1);
	}



	/*
	*The API clCreateCommandQueue creates a command-queue on a specific device.
	*/
	commandQueue = clCreateCommandQueueWithProperties(context, device, 0, &status);
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Creating command queue failed!" << std::endl;
		exit(1);
	}


	//create  kernel	
	kernel = clCreateKernel(program, "Compare", &status);
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Creating kernel1 failed!" << std::endl;
		exit(1);
	}

	//create  kernel	
	/*kernel2 = clCreateKernel(program2, "CreateMap", &status); 
		if (status != CL_SUCCESS)
		{
			std::cout << "Error: Creating kernel2 failed!" << std::endl;
			exit(1);
		}
*/

	//set kernel args.
	status = clSetKernelArg(kernel, 0, sizeof(cl_mem), &inputBuffer);
	status = clSetKernelArg(kernel, 1, sizeof(cl_mem), &input2Buffer);
	status = clSetKernelArg(kernel, 2, sizeof(cl_mem), &outputBuffer);
	/*
	status = clSetKernelArg(kernel2, 0, sizeof(cl_mem), &input3Buffer);
	status = clSetKernelArg(kernel2, 1, sizeof(cl_mem), &outputBuffer);

	*/



}





//Clean up the resourses - called when GPU_SLAM should be shut down
void GPU_SLAM::cleanUp() {
	status = clReleaseKernel(kernel);
	//status = clReleaseKernel(kernel2);
	status = clReleaseMemObject(inputBuffer);//Release mem object.
	status = clReleaseMemObject(outputBuffer);

	status = clReleaseProgram(program);//Release program2.
	//status = clReleaseProgram(program2);//Release program2.
	status = clReleaseCommandQueue(commandQueue);//Release command queue.
	status = clReleaseContext(context);//Release context.
}

int GPU_SLAM::convertToString(const char* filename, std::string& s)
{
	size_t size;
	char* str;
	std::fstream f(filename, (std::fstream::in | std::fstream::binary));

	if (f.is_open())
	{
		size_t fileSize;
		f.seekg(0, std::fstream::end);
		size = fileSize = (size_t)f.tellg();
		f.seekg(0, std::fstream::beg);
		str = new char[size + 1];
		if (!str)
		{
			f.close();
			return 0;
		}

		f.read(str, fileSize);
		f.close();
		str[size] = '\0';
		s = str;
		delete[] str;
		return 0;
	}
	std::cout << "Error: failed to open file\n:" << filename << std::endl;
	return FAILURE;
}

//create gpu device
cl_device_id GPU_SLAM::create_device() {

	cl_platform_id platform;
	cl_device_id dev;
	int err;

	/* Identify a platform */
	err = clGetPlatformIDs(1, &platform, NULL);
	if (err < 0) {
		perror("Couldn't identify a platform");
		exit(1);
	}

	// Access a device
	// GPU
	err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &dev, NULL);
	if (err == CL_DEVICE_NOT_FOUND) {
		// CPU
		err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 1, &dev, NULL);
	}
	if (err < 0) {
		perror("Couldn't access any devices");
		exit(1);
	}

	return dev;
}

/* Create program from a file and compile it */
cl_program GPU_SLAM::build_program(cl_context ctx, cl_device_id dev, const char* filename) {

	cl_program program;
	//FILE* program_handle;
	char* program_buffer, * program_log;
	size_t program_size, log_size;
	int err;

	std::string sourceStr;

	convertToString(filename, sourceStr);
	const char* source = sourceStr.c_str();
	program_size = strlen(source);


	/* Create program from file

	Creates a program from the source code in the add_numbers.cl file.
	Specifically, the code reads the file's content into a char array
	called program_buffer, and then calls clCreateProgramWithSource.
	*/
	program = clCreateProgramWithSource(ctx, 1, &source, &program_size, &err);
	if (err < 0) {
		perror("Couldn't create the program");
		exit(1);
	}
	//free(program_buffer);

	/* Build program

	The fourth parameter accepts options that configure the compilation.
	These are similar to the flags used by gcc. For example, you can
	define a macro with the option -DMACRO=VALUE and turn off optimization
	with -cl-opt-disable.
	*/
	err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
	if (err < 0) {

		/* Find size of log and print to std output */
		clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
			0, NULL, &log_size);
		program_log = (char*)malloc(log_size + 1);
		program_log[log_size] = '\0';
		clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
			log_size + 1, program_log, NULL);
		printf("%s\n", program_log);
		free(program_log);
		exit(1);
	}

	return program;
}




float GPU_SLAM::getScore(Offset offset) {
	MiniMap m(map);
	Pose RobotPose = robot->getCorrectedPose();
	Pose newPose;


	newPose.x = RobotPose.x + offset.x;
	newPose.y = RobotPose.y + offset.y;
	newPose.theta = RobotPose.theta + offset.theta;
	
	m.createQuickMiniMap(data->getFrameWithOffset(offset), newPose);
	//m.createMiniMap(data->getFrameWithOffset(offset), newPose);


	//auto start = high_resolution_clock::now();
	createInput(m.getMap());
	//auto start2 = high_resolution_clock::now();
	float bestScore = runGPU();
	//now add distance to score
	bestScore = 0.99 * bestScore + 0.1 * (std::abs(offset.x) + std::abs(offset.y) + std::abs(offset.theta));
	
	
	//auto stop = high_resolution_clock::now();
	//auto duration1 = duration_cast<microseconds>(start2 - start);
	//auto duration2 = duration_cast<microseconds>(stop - start2);
	//std::cout << "input data: " << duration1.count() << std::endl;
	//std::cout << "getscoretime Time: " << duration2.count() << std::endl;
	return bestScore;
}

void GPU_SLAM::createInput(std::vector < MMTile> miniMap) {
	
	if (miniMap.size() > GlobalThreadSize) {
		std::cout << "increase GlobalThreadSize" << std::endl;
		throw "increse GlobalThreadSize";
	}
	
	
	for (int i = 0; i < GlobalThreadSize; i++) {
		if (i < miniMap.size()) {
			float mapProb = map->getOccupiedTileProb(miniMap[i].x, miniMap[i].y);
			//float mapProb = 0.1;
			input[i] =  mapProb;
			input2[i] = miniMap[i].occupied;
			//std::cout << input[i] << "  " << input2[i] << std::endl;
		}
		else {
			input[i] = 0.0;
			input2[i] = 0.0;
		}
		
		
	}

}

float GPU_SLAM::runGPU() {


	//create dummy data - for testing
	//for (int i = 0; i < GlobalThreadSize ; i++)
	//{
	//	input[i] = test;
	//	input2[i] = test;
	//}
	//auto start = high_resolution_clock::now();
	status = clEnqueueNDRangeKernel(commandQueue, kernel, 1, NULL, &global_threads, NULL, 0, NULL, NULL); //input
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Enqueue kernel onto command queue failed!" << std::endl;
		exit(1);
	}

	// Read the kernel's output    
	status = clEnqueueReadBuffer(commandQueue, outputBuffer, CL_TRUE, 0,
		sizeof(output), output, 0, NULL, NULL); // <=====GET OUTPUT
	if (status != CL_SUCCESS) {
		perror("Couldn't read the buffer");
		exit(1);
	}

	long float sum = 0;
	int count = 0;
	for (int i = 0; i < GlobalThreadSize; i++) {
		//std::cout << "diff " << std::to_string(output[i]) << std::endl;
		float t = output[i];
		if (t < 10.0) {
			sum += t;
			count++;
		}
		
	}
	sum = sum / (float)count;
	//time
	//auto stop = high_resolution_clock::now();
	//auto duration = duration_cast<microseconds>(stop - start);
	//std::cout <<"Time: " << duration.count() << std::endl;

	//std::cout << "sum: " << std::to_string(sum) << std::endl;

	return sum;

}
/*
void GPU_SLAM::createInputForMap() {




	for (int i = 0; i < 363; i++) {
		//input3[i] = 0.0001;


	}

}


float GPU_SLAM::runGPUMapCreation() {


	//create dummy data - for testing
	//for (int i = 0; i < GlobalThreadSize ; i++)
	//{
	//	input[i] = test;
	//	input2[i] = test;
	//}
	auto start = high_resolution_clock::now();
	//status = clEnqueueNDRangeKernel(commandQueue, kernel2, 1, NULL, &global_threads, NULL, 0, NULL, NULL); //input
	if (status != CL_SUCCESS)
	{
		std::cout << "Error: Enqueue kernel onto command queue failed!" << std::endl;
		exit(1);
	}

	// Read the kernel's output    
	status = clEnqueueReadBuffer(commandQueue, outputBuffer, CL_TRUE, 0,
		sizeof(output), output, 0, NULL, NULL); // <=====GET OUTPUT
	if (status != CL_SUCCESS) {
		perror("Couldn't read the buffer");
		exit(1);
	}

	long float sum = 0;
	int count = 0;
	for (int i = 0; i < GlobalThreadSize; i++) {
		//std::cout << "diff " << std::to_string(output[i]) << std::endl;
		float t = output[i];
		sum += t;
	}
	
	//time
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop - start);
	//std::cout <<"Time: " << duration.count() << std::endl;

	std::cout << "sum: " << std::to_string(sum) << std::endl;

	return sum;

}
*/
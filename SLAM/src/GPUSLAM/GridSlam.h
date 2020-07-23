//GPUGM-SLAM manager
#pragma once

#include "../Map/Map.h"
#include "../ImportData/Constants.h"
#include "MiniMap.h"
#include "../Robot/Robot.h"
#include "../Interface.h"
#include "../Data.h"
#include "../Log/Logging.h"
#include "Hillclimb.h"
#include "GPU_SLAM.h"
#include "Random.h"

#include <CL/cl.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>

#include <chrono> //time

using namespace std::chrono;

class GridSlam
{
	Map* map;
	Data* data;
	Robot* robot;

	GPU_SLAM gpu;
	Offset lastOffset;

public:
	GridSlam(Map* newmap, Data* newdata, Robot* newrobot);
	void runNextFrame();//run next frame with GPUGM-SLAM
	void setUp();//set up gpu
	void cleanUp();//clean up gpu after slam runthorugh
	Offset getRecomendedOffset();//get offset recomended by GPUGM-SLAM
};


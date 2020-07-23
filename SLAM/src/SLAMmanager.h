//main SLAM loop - manages which base slam algorithm to run
#pragma once
#include "Robot/Robot.h"
#include "Data.h"
#include "Map/Map.h"
#include "OrthoSLAM/OrthoSLAM.h"
#include "GPUSLAM/GridSlam.h"

class SLAMmanager
{
	Map* map;
	Data* data;
	Robot* robot;
	GridSlam* gridSlam;
	OrthoSLAM* orthoSlam;
	Logger* log;
	double alpha = 1.0;
	double beta = 1;
	double gamma = 1;
	double delta= 0.000050;
	int minFrames = 5;

	bool orthoSlamRunning = true;
	int frameCount = 0;

	double pScore = 0;
	
	
public:
	SLAMmanager(Logger* newlog,Map* newmap, Data* newdata, Robot* newrobot, GridSlam* newGridSlam, OrthoSLAM* newOrthoSlam);
	void setup(double alpha, double beta, double gamma, double delta, int minFrames);
	void run();//run the main loop
private:
	bool runGPUslamNextFrame(int linesDetected, int linesAligned, Offset OrthoOffset, Offset GPUoffset);//should algorithm run gpu slam next frame?
	bool runGPUslamNextFrame(int linesDetected, int linesAligned, Offset OrthoOffset);//should algorithm run gpu slam next frame?

};


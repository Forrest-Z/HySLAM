//this class manages the data storage and streaming to two base slam algorithms 
#pragma once

#define _USE_MATH_DEFINES

#include "ImportData/ImportLidar.h"
#include "Structures.h"
#include "Robot/Robot.h"
#include "Map/Map.h"
#include <cmath>
#include "Interface.h"
#include "Map/RenderMap.h"
#include<string>


class Data
{
	std::vector<LDataLine> data;
	std::vector<LDataLine> correctedData;//full list af every frame of data
	unsigned int count = 0;//curent frame of data
	unsigned int endframe = 999999;//last frame of data to stop on
	Robot* robot;
	Map* map;
	Logger* log;

	LDataLine rawDataCurrentFrame;
	FrameCoords currentFrame;

	int test = 0;

public:

	//these have been moved to config
	//bool applyCorectionOrthoSlam = true;
	//bool applyCorectionGridSlam = true;
	//bool OrthoSLAMWriteToRobotMap = true;
	//bool runOnlyOrthoSLAM = true; // used only when applycorrection  == false

	Data(Logger* newlog, Robot* robot, Map* map);
	void import();
	void setNextFrame();//calcualte the next frame anf set as current frame - should be called before getFrame and getFrameWithOffset.
	FrameCoords getFrame();//get current frame
	FrameCoords getFrameWithOffset(Offset offset);//get current frame with an offset applied
	FrameCoords getFrameWithOffset(Offset offset, float maxRange);//get current frame with an offset applied - also override config max lidar range - used for quick minimap generation gpu slam
	void import(std::string fileName, std::string number);//import - used for looping
	void historyRollBack();
private:
	FrameCoords convertToWorldCoords(LDataLine rawData);
	void importCorrectedData();
	void combineCorrectedData();
	void write(std::string fileName);

	std::vector<double> noiseRemoval(std::vector<double> lidar);

};


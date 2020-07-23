//the robot class manages the robot pose 
#pragma once
#include "../Structures.h"
#include "../Log/Logging.h"
#include <iostream>
#include "../Map/Map.h"

class Robot
{
	Pose rawPose;
	Pose correctedPose;
	Offset offset;

	Logger* log;
	Map* map;

public:
	Robot(Logger* newlog, Map* newmap);
	
	void setRawPose(double x, double y, double theta );
	void addOffset(Offset newoffset);
	Pose getCorrectedPose();//calualte corrected pose based one raw pose and offset
	void setTime(double newTime);

	void historyRollBack(Offset historyOffset);//role back offset - remove history offset from robot offset

private:
	void updateCorrectedPose();

};


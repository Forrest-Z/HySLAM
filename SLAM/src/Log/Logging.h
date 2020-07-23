//logger class - manages the log file
#pragma once

#include <string>
// basic file operations
#include <iostream>
#include <fstream>
#include "../Structures.h"


class Logger
{
	double time = 0.0;
	std::string SLAMrunning = "unknown";
	std::string fileName =  "log.txt";

	Pose rawPose;
	Pose correctedPose;
	Offset offset;

public:

	Logger();
	void setLogName(std::string fileName);//set the name of the log file, does not need file extention, defults to log.txt
	void setTime(double time);//set timestamp time
	void log(int linesDetected, int linesAligned, double pScore, std::string frametime);//set frame details
	void logString(std::string info);//add note to log
	void loginfo(float inc, float score);
	void setSLAM(std::string SLAMrunning);//set type of slam running - GPU or ortho
	void setRobotPose(Pose rawPose, Pose correctedPose, Offset offset);//set the robot pose of the robot in log file

};


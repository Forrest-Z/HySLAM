/*
	Heterogeneous-SLAM: SLAM algorithm
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

#include "Robot.h"


Robot::Robot(Logger* newlog, Map* newmap)
{
	log = newlog;
	map = newmap;
	Pose i;
	i.x = 0.0;
	i.y = 0.0;
	i.theta = 0.0;
	rawPose = i;
	correctedPose = i;


}

void Robot::historyRollBack(Offset historyOffset) {
	offset.x = historyOffset.x;
	offset.y = historyOffset.y;
	offset.theta = historyOffset.theta;
	updateCorrectedPose();
}

void Robot::setRawPose(double x, double y, double theta) {
	rawPose.x = x;
	rawPose.y = y;
	rawPose.theta = theta;
	updateCorrectedPose();
}

void Robot::updateCorrectedPose() {
	correctedPose.x = rawPose.x + offset.x;
	correctedPose.y = rawPose.y + offset.y;
	correctedPose.theta = rawPose.theta + offset.theta;
}

void Robot::addOffset(Offset newoffset) {

	map->addOffsetToHistory(offset);

	offset.x += newoffset.x;
	offset.y += newoffset.y;
	offset.theta += newoffset.theta;

	

	updateCorrectedPose();

	log->setRobotPose(rawPose, correctedPose, offset);
	/*
	std::cout << "raw x: " << rawPose.x << std::endl;
	std::cout << "cor x: " << correctedPose.x << std::endl;
	std::cout << "off x: " << offset.x << std::endl;
	std::cout << "raw y: " << rawPose.y << std::endl;
	std::cout << "cor y: " << correctedPose.y << std::endl;
	std::cout << "off y: " << offset.y << std::endl;
	std::cout << "raw theta: " << rawPose.theta << std::endl;
	std::cout << "cor theta: " << correctedPose.theta << std::endl;
	std::cout << "off theta: " << offset.theta << std::endl;
	*/
}

Pose Robot::getCorrectedPose() {
	return correctedPose;
}

void Robot::setTime(double newtime) {
	log->setTime(newtime);
}
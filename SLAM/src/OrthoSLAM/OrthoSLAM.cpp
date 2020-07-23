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

#include "OrthoSLAM.h"




OrthoSLAM::OrthoSLAM(Map *newmap, Data *newdata, Robot* newrobot) {

	map = newmap;
	data = newdata;
	robot = newrobot;
	
	lineDetection.setup(data,robot,map);


	MAX_ERROR_TILE = (int)((map->config.MAX_ERROR / map->config.TILE_SIZE) + 0.5);
	MIN_OVERLAP_TILE = (int)((map->config.MIN_OVERLAP / map->config.TILE_SIZE) + 0.5);
	if (MIN_OVERLAP_TILE == 0) {
		MIN_OVERLAP_TILE = 1;
	}

	std::cout << "MIN_OVERLAP_TILE: " << MIN_OVERLAP_TILE << std::endl;
	std::cout << "MAX_ERROR_TILE: " << MAX_ERROR_TILE << std::endl;
	
}


void OrthoSLAM::runNextFrame() {
	//get lines and add to current frame (this should only get lines that ae orthogonal)
	//get offset for each line - add to total offset
	//add offset for each line
	//add to map
	//clear frame

	//first reset counts
	currentFrame.clear();

	//reset the offset and count
	frameOffset.x = 0.0;
	frameOffset.y = 0.0;
	frameOffset.theta = 0.0;
	countX = 0;
	countY = 0;
	countTheta = 0;

	linesAligned = 0;
	linesDetected = 0;



	
	if (firstFrame == true && map->config.applyCorectionOrthoSlam == true) {
		while (currentFrame.size() == 0) {
			currentFrame = lineDetection.getStartFrame();
		}
		linesDetected = currentFrame.size();

		//get offset and add to each line 
		
		double rot = getStartRotationalOffset();
		std::cout << "start rotation offset: " << rot << std::endl;
		frameOffset.theta = rot;
		
		//add offset to robot - also resets offset  
		addOffsetToRobot();

		//clear frame
		currentFrame.clear();
		firstFrame = false;
	}
	else {

	
	//get lines
	getNextFrame();
	//get offset and add to each line 
	if (map->config.applyCorectionOrthoSlam == true) {
		double rot = getRotationalOffset();
		rotateFrame(rot);
		frameOffset.theta = rot;
		offsetCurrentFrame();//x and y
	}
	//add lines to a map
	//addLinesToMap();
	//add offset to robot - also resets offset
	
	addOffsetToRobot();
	createMiniMap(frameOffset);
	addLinesToMap();
	//clear frame
	

	}
}

void OrthoSLAM::addOffsetToRobot() {
	if (countY > 0) {
		frameOffset.y = frameOffset.y / countY;
	}
	if (countX > 0) {
		frameOffset.x = frameOffset.x / countX;
	}
	if (map->config.OrthoSLAMWriteToRobotMap == true) {
		robot->addOffset(frameOffset);
	}
	

	//std::cout << "offsetframe count: x: " << countX << " y: " << countY << " theta: " << countTheta << std::endl;
	//std::cout << "offsetframe : x: " << frameOffset.x << " y: " << frameOffset.y << " theta: " << frameOffset.theta << std::endl;


	
}

void OrthoSLAM::createMiniMap(Offset offset) {
	if (map->config.OrthoSLAMWriteToRobotMap == true) {
		Pose RobotPose = robot->getCorrectedPose();
		Pose newPose;
		newPose.x = RobotPose.x + offset.x;
		newPose.y = RobotPose.y + offset.y;
		newPose.theta = RobotPose.theta + offset.theta;
		MiniMap m(map);
		m.createMiniMapNoBlur(data->getFrameWithOffset(offset), RobotPose);
		//m.createQuickMiniMap(data->getFrameWithOffset(offset), newPose);
		map->addMiniMap(m.getMap());
	}
}

void OrthoSLAM::getNextFrame() {
	//get lines add to current frame
	currentFrame = lineDetection.getNextFrame();
	//linesDetected = currentFrame.size();
	for (int i = 0; i < currentFrame.size(); i++) {
		Line line = currentFrame[i];
		if (line.direction == Direction::Horizontal) {
			double length = std::abs(line.a.x - line.b.x);
			int tileLength = (int)((length / map->config.TILE_SIZE) + 0.5);
			linesDetected += tileLength;
		}
		else {
			double length = std::abs(line.a.y - line.b.y);
			int tileLength = (int)((length / map->config.TILE_SIZE) + 0.5);
			linesDetected += tileLength;
		}
	}
	std::cout << "line size:  " << currentFrame.size() << std::endl;
}

//add a line to the map
void OrthoSLAM::addLinesToMap() {
	if (map->config.OrthoSLAMWriteToRobotMap == true) {
		for (int i = 0; i < currentFrame.size(); i++) {
			Line line = currentFrame[i];
			if (line.direction == Direction::Vertical) {
				int lineX = map->convertCoords(line.centre.x) ;
				int maxY = map->convertCoords(line.b.y + frameOffset.y);//add in frame offset
				int minY = map->convertCoords(line.a.y + frameOffset.y);
				for (int y = minY; y < maxY; y++) {
					map->tileSeen(lineX, y, line.facing);
				}
			}
			else {
				int lineY = map->convertCoords(line.centre.y ) ;
				int maxX = map->convertCoords(line.b.x + frameOffset.x);
				int minX = map->convertCoords(line.a.x + frameOffset.x);
				for (int x = minX; x < maxX; x++) {
					map->tileSeen(x, lineY, line.facing);
				}
			}
		}
	}
	
}

void OrthoSLAM::addSwitchLockPoint() {
	addLinesToMap();
	addLinesToMap();
	addLinesToMap();
	addLinesToMap();
	addLinesToMap();
}

//get offset and add to each line
void OrthoSLAM::offsetCurrentFrame() {
	
	for (int i = 0; i < currentFrame.size(); i++) {
		Offset offset = getLineOffset(currentFrame[i]);
		if (currentFrame[i].direction == Direction::Vertical) {
			currentFrame[i].a.x += offset.x;
			currentFrame[i].b.x += offset.x;
			currentFrame[i].centre.x += offset.x;
			frameOffset.x += offset.x;
			if (std::abs(offset.y) > 0.1) {//count number of lines that align with main map - y offset used as indicator of allignment
				//linesAligned++;
			}
			countX++;
		}
		else {
			currentFrame[i].a.y += offset.y;
			currentFrame[i].b.y += offset.y;
			currentFrame[i].centre.y += offset.y;
			frameOffset.y += offset.y;
			if (std::abs(offset.x) > 0.1) {//count number of lines that align with main map - x offset used as indicator of allignment
				//linesAligned++;
			}
			countY++;
		}
	}
}

//calcualte the offset for a line.
//will find nearest landmarks and shift line to landmark
Offset OrthoSLAM::getLineOffset(Line line) {
	if (line.direction == Direction::Horizontal){//if line horizontal - will only look at the landmarks abouve and below
		
		return getHorizontalLineOffset(line);
	}
	else { // line vertical - only lokk at land marks left and right

		return getVerticalLineOffset(line);
	}
	
}

//calcualte offset for horsizontal line
Offset OrthoSLAM::getHorizontalLineOffset(Line line) {
	if (line.direction != Direction::Horizontal) {
		throw "wrong direction! line should be Horizontal (OrthoSLAM.cpp getHorizontalLineOffset())";
	}

	int lineY = map->convertCoords(line.centre.y);
	int maxX = map->convertCoords(line.b.x);
	int minX = map->convertCoords(line.a.x);

	//std::cout << "liny: " << lineY << std::endl;
	//std::cout << "maxx: " << maxX << std::endl;
	//std::cout << "minx: " << minX << std::endl;
	//std::cout << "TILEERROR: " << MAX_ERROR_TILE << std::endl;
	Offset offset;
	offset.x = 0.0;
	offset.y = 0.0;
	double posativeDistance = 0.0;
	double negativeDistance = 0.0;

	int limit = (maxX - minX)/2;
	//std::cout << "limit: " << limit << std::endl;
	
	int start = minX + (limit);
	//std::cout << "start: " << start << std::endl;

	int negCounter = 0;
	int posCounter = 0;

	for (int i = 0; i <= MAX_ERROR_TILE; i++) {
		int y;
		int x;
		if (i == 0) {
			y = lineY;
			int np = loopHorizontal(y, start, limit, line.facing);
			if (np >= 1) {
				negCounter = np ;
				posCounter = np ;
				offset.y = map->convertCoords(y) - line.centre.y;
				offset.x = 1.0;//used as indicator of alignment
			}
		}

		y = lineY + i; //look abouve for landmark
		int p = loopHorizontal(y, start, limit, line.facing);
		if (p > posCounter) {
			posCounter = p ;
			posativeDistance = map->convertCoords(y) - line.centre.y;
		}

		y = lineY - i; //look below
		int n = loopHorizontal(y, start, limit, line.facing);
		if (n > negCounter) {
			negCounter = n ;
			negativeDistance = map->convertCoords(y) - line.centre.y;
		}
	}
	std::cout << "counter " << negCounter << " " << posCounter << std::endl;
	
	//check if any landmarks found i.e. dist is non zero
	if (posativeDistance != 0.0 || negativeDistance != 0.0) {
		offset.x = 1.0;//used as indicator of alignment
		if (posativeDistance == 0.0) {//which is zero - return non zero dist
			offset.y = negativeDistance;
			linesAligned += negCounter;
		}
		else if (negativeDistance == 0.0) {
			offset.y = posativeDistance;
			linesAligned += posCounter;
		}
		else if (std::abs(negativeDistance) < posativeDistance) { // if both are non zero return smallest
			offset.y = negativeDistance;
			linesAligned += negCounter;
		}
		else {
			offset.y = posativeDistance;
			linesAligned += posCounter;
		}
		std::cout << "offset " << offset.y << std::endl;
		return offset;
	}
	std::cout << "offset " << offset.y << std::endl;
	
	return offset;
}

//loop and find lines along y row
// will start at startpoint and look each side by range.
int OrthoSLAM::loopHorizontal(int y, int startPointX, int range, Facing facing) {
	int x;
	int count = 0;
	for (int j = 0; j <= range; j++) {
		x = startPointX + j;
		if (map->isLandmark(x, y, facing) == true) {
			count++;
		}
		x = startPointX - j;
		if (map->isLandmark(x, y, facing) == true) {
			count++;
		}

		
	}
	if (count >= MIN_OVERLAP_TILE) {
		return count;
	}
	return 0;
}


//calcualte offset for vertical line
Offset OrthoSLAM::getVerticalLineOffset(Line line) {
	if (line.direction != Direction::Vertical) {
		throw "wrong direction! line should be Vertical (OrthoSLAM.cpp getVerticalLineOffset())";
	}

	int lineX = map->convertCoords(line.centre.x);
	int maxY = map->convertCoords(line.b.y);
	int minY = map->convertCoords(line.a.y);

	//std::cout << "linX: " << lineX << std::endl;
	//std::cout << "maxY: " << maxY << std::endl;
	//std::cout << "minY: " << minY << std::endl;
	//std::cout << "TILEERROR: " << MAX_ERROR_TILE << std::endl;
	Offset offset;
	offset.x = 0.0;
	offset.y = 0.0;
	double posativeDistance = 0.0;
	double negativeDistance = 0.0;

	int limit = (maxY - minY) / 2;
	//std::cout << "limit: " << limit << std::endl;

	int start = minY + (limit);
	//std::cout << "start: " << start << std::endl;

	int negCounter = 0;
	int posCounter = 0;

	for (int i = 0; i <= MAX_ERROR_TILE; i++) {
		int y;
		int x;
		if (i == 0) {
			x = lineX;
			int np = loopVertical(x, start, limit, line.facing);
			if (np >= 1) {
				negCounter = np ;
				posCounter = np ;
				offset.x = map->convertCoords(x) - line.centre.x;
				offset.y = 1.0;//used as indicator of alignment
			}
		}

		x = lineX + i; //look abouve for landmark
		int p = loopVertical(x, start, limit, line.facing);
		if (p > posCounter) {
			posCounter = p ;
			posativeDistance = map->convertCoords(x) - line.centre.x;
		}

		x = lineX - i; //look below
		int n = loopVertical(x, start, limit, line.facing);
		if (n > negCounter) {
			negCounter = n ;
			negativeDistance = map->convertCoords(x) - line.centre.x;
		}
	}
		//check if any landmarks found i.e. dist is non zero
		if (posativeDistance != 0.0 || negativeDistance != 0.0) {
			offset.y = 1.0;//used as indicator of alignment
			if (posativeDistance == 0.0) {//which is zero - return non zero dist
				offset.x = negativeDistance;
				linesAligned += negCounter;
			}
			else if (negativeDistance == 0.0) {
				offset.x = posativeDistance;
				linesAligned += posCounter;
			}
			else if (std::abs(negativeDistance) < posativeDistance) { // if both are non zero return smallest
				offset.x = negativeDistance;
				linesAligned += negCounter;
			}
			else {
				offset.x = posativeDistance;
				linesAligned += posCounter;
			}
			
			return offset;
		}
	
	
	return offset;
}

// loop along column x
// will start at startpoint and look each side by range.
int OrthoSLAM::loopVertical(int x, int startPointY, int range, Facing facing) {
	int y;
	int count = 0;
	for (int j = 0; j <= range; j++) {
		y = startPointY + j;
		//std::cout << "x: " << x << " y: " << y << std::endl;
		if (map->isLandmark(x, y,facing) == true) {
			count++;
		}
		y = startPointY - j;
		//std::cout << "x: " << x << " y: " << y << std::endl;
		if (map->isLandmark(x, y,facing) == true) {
			count++;
		}

		
	}
	if (count >= MIN_OVERLAP_TILE) {
		return count;
	}
	return 0;
}

//get the rotational offset - i is the index of the line in current frame
double OrthoSLAM::getRotationalOffset() {
	//posative rotation = anti clockwise
	double averageRot = 0.0;
	double oldaverageRot = 0.0;
	double totalLength = 0.0;
	if (currentFrame.size() > 0) {
		for (int i = 0; i < currentFrame.size(); i++) {
			Line line = currentFrame[i];
			if (line.direction == Direction::Vertical) {
				double ajecent = line.b.y - line.a.y;
				double opposit = line.b.x - line.a.x;
				double theta = std::atan2(opposit, ajecent);
				//try weighting lines on length
				oldaverageRot += theta;
				theta = theta * ajecent;// *ajecent;
				totalLength += ajecent;// *ajecent;
				averageRot += theta;
			}
			else {
				double ajecent = line.b.x - line.a.x;
				double opposit = line.a.y - line.b.y;
				double theta = std::atan2(opposit, ajecent);
				oldaverageRot += theta;
				theta = theta * ajecent;// *ajecent;
				totalLength += ajecent;// *ajecent;
				averageRot += theta;
			}
		}
		averageRot = averageRot / totalLength;
		oldaverageRot  = oldaverageRot / (double)currentFrame.size();
	}
	//std::cout << "old angle: " << oldaverageRot << " new ang: " << averageRot << std::endl;
	return oldaverageRot;//this give beter result than averageangle it seams - mor etesting
}

void OrthoSLAM::rotateFrame(double rotation) {
	
	Pose center = robot->getCorrectedPose();

	for (int i = 0; i < currentFrame.size(); i++) {
		Point point = currentFrame[i].a;

		double rotatedX = std::cos(rotation) * (point.x - center.x) - std::sin(rotation) * (point.y - center.y) + center.x;
		double rotatedY = std::sin(rotation) * (point.x - center.x) + std::cos(rotation) * (point.y - center.y) + center.y;
		currentFrame[i].a.x = rotatedX;
		currentFrame[i].a.y = rotatedY;

		point = currentFrame[i].b;
		rotatedX = std::cos(rotation) * (point.x - center.x) - std::sin(rotation) * (point.y - center.y) + center.x;
		rotatedY = std::sin(rotation) * (point.x - center.x) + std::cos(rotation) * (point.y - center.y) + center.y;
		currentFrame[i].b.x = rotatedX;
		currentFrame[i].b.y = rotatedY;
	}


	
}

double OrthoSLAM::getStartRotationalOffset() {
	//posative rotation = anti clockwise
	double averageRot = 0.0;
	if (currentFrame.size() > 0) {
		for (int i = 0; i < currentFrame.size(); i++) {
			Line line = currentFrame[i];
			if (line.direction == Direction::Vertical) {
				double ajecent = line.b.y - line.a.y;
				double opposit = line.b.x - line.a.x;
				if (ajecent > 1.0) {
					double theta = std::atan2(opposit, ajecent);
					averageRot += theta;
				}
				
			}
			else {
				double ajecent = line.b.x - line.a.x;
				double opposit = line.a.y - line.b.y;
				if (ajecent > 1.0) {
					double theta = std::atan2(opposit, ajecent);
					averageRot += theta;
				}
				
			}
		}

		averageRot = averageRot / (double)currentFrame.size();
	}

	return averageRot;
}


int OrthoSLAM::getLinesDetectedCount() {

	return linesDetected;
}
int OrthoSLAM::getLinesAlignedCount() {

	return linesAligned;
}
Offset OrthoSLAM::getFrameOffset() {
	return frameOffset;
}
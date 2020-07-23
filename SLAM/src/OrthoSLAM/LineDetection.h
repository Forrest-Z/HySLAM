#pragma once

#include "../ImportData/LidarData.h"
#include "../Data.h"
#include "../Structures.h"
#include "../Map/Map.h"
#include <vector>
#include <cmath>

class LineDetection
{
	Data* data;
	Robot* robot;
	Map* map;

	double maxDisBetweenPoints = 0.0; // the maximum distance allowed between points in line - calcuated in constructor for this class.
	
	int counter = 0;//track wich lidar point on


public:
	LineDetection();
	void setup(Data* newdata, Robot* newrobot, Map* newmap);
	std::vector<Line> getNextFrame();//get next frame of data
	std::vector<Line> getStartFrame();

	
private:
	std::vector<Line> extractLines(FrameCoords newdata);//extract lines from data
	Direction isOrthogonal(Point start, Point end, int startIndex, int endIndex);//check line is orthogonal to map
	bool isInLine(Point start, Point check, Direction direction);// check if point is inline with stsrt point

	std::vector<Line> extractStartLines(FrameCoords newdata);
	Direction isStartLine(Point start, Point end, int startIndex, int endIndex);
	
	Line createLine(Point start, Point end, Direction dir);//create line between points
};


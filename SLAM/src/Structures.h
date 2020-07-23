//defines all standeard structures used in program
#pragma once
#include <vector>
#include "Map/Tile.h"

struct Point {// coordinate
	double x = 0.0;
	double y = 0.0;
};

enum class Direction { Vertical, Horizontal, None, Unkown };// directions of lines

struct Line {// a line for orthoSLAM
	int id = 0;
	// point a is always the left or bottom most point in a line
	Point a; 
	// point b is always the right or upper point in a line
	Point b; 
	//center of line 
	Point centre;

	Direction direction = Direction::None;
	//int timesObserved = 0;
	//double timeSinceLastSeen = 0.0;
	Facing facing = Facing::N;
	
};


struct Offset { //offset applied to robot / frame
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
};

struct FrameCoords {//list of raw data points
	std::vector<Point> points;
};

struct Pose {//robot pose
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
};
//main OrthoSLAM class
#pragma once
#include "../Structures.h"
#include "../Map/Map.h"
#include "../Data.h"
#include "LineDetection.h"
#include "../Robot/Robot.h"
#include "../GPUSLAM/MiniMap.h"
#include <vector>



class OrthoSLAM
{
	int MAX_ERROR_TILE; // derived from the MAX_ERROR and TILE_SIZE
	int MIN_OVERLAP_TILE; // derived from MIN_OVERLAP and TILE_SIZE

	Map *map;
	Data* data;
	Robot* robot;
	LineDetection lineDetection;

	std::vector<Line> currentFrame;//lines found in current frame
	Offset frameOffset;//recomended frame offset
	bool firstFrame = true;

	int countX = 0;//used to calc average offset in each direction
	int countY = 0;
	int countTheta = 0;


	int linesDetected = 0;
	int linesAligned = 0;



public:
	OrthoSLAM(Map* newmap, Data* newdata, Robot* newrobot);

	void runNextFrame();//use ORthoSLAm to run next data frame
	int getLinesDetectedCount();
	int getLinesAlignedCount();
	Offset getFrameOffset();

	void addSwitchLockPoint();

private:
	Offset getLineOffset(Line line);//calculate line offset - data assosiation
	Offset getHorizontalLineOffset(Line line);
	int loopHorizontal(int y, int startPointX, int range, Facing facing);
	Offset getVerticalLineOffset(Line line);
	int loopVertical(int y, int startPointX, int range, Facing facing);

	void offsetCurrentFrame();//calc offset 

	void addLinesToMap();//add offset ligts to absolute map
	
	void getNextFrame();

	double getRotationalOffset();
	void addOffsetToRobot();

	void rotateFrame(double rotation);

	double getStartRotationalOffset();

	void createMiniMap(Offset offset);//create local view map

	
};


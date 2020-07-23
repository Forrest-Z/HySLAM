//gradient desent algorithm for GPUGM-SLAM
//will cheack in every direction untill lower score is found then move to that offset and double step size, if no better score half step size try all directions again

#pragma once
#include <vector>

#include "../Structures.h"
#include "GPU_SLAM.h"
#include "Random.h"


class Hillclimb
{
	Map* map;
	GPU_SLAM* gpu;
	float currentScore = 1.0;//current best score seen, want to minimise
	Random random;//class that handels random numbers
	float stepSize = 0.05;//amount to change offset by each iteration, changes throughout runthough
	Offset currentOffset;//current best offset - lowest score
	bool offsetChecked[6] = { false,false,false,false, false,false };// true when direction checked, (x+, x-, y+, y-, theta+ , theta-)

	double minDist = 0.01;//set to map tile size / 5 in constructor
	double thetaStepReduction = 8.0;//set to max lidar range in constructor

public:
	Hillclimb(Map* newmap, GPU_SLAM* gpu);
	Offset climb(Offset prevOffset);//start gradient descent, will run through twice once with prevoffset as starting pose and the other with 0,0,0 as starting pose - will retrun the best offset achived across both.
	Offset climber(Offset Offset);//start gradient descent, with a starting pose of offset 
private:

	
	void incStep();//increase step size (double)
	void decStep();//decrease step size (half)
	Offset getNextOffset();//calualte the next offset to try
	bool hasBeenChecked(int direction);//has givent direction been tried, 1-6
	void resetCheckedOffset();//usaed after step size changes - reset the directions checked


};


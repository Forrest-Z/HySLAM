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

#include "Hillclimb.h"

Hillclimb::Hillclimb(Map* newmap, GPU_SLAM* newGpu) {
	map = newmap;
	gpu = newGpu;
	

	thetaStepReduction = map->config.MAX_LIDAR_RANGE;
	minDist = map->config.TILE_SIZE / 5.0;
	//std::cout << "mindist " << minDist << std::endl;
	//std::cout << "stepreduction " << thetaStepReduction << std::endl;
}


Offset Hillclimb::climb(Offset prevOffset) {
	Offset firstOffset = climber(prevOffset);//try with last offset
	float firstScore = currentScore;
	Offset noOffset;
	Offset secondOffset = climber(noOffset);//now try with 0 offsret take best
	if (firstScore < currentScore) {
		//std::cout << "first "<< firstScore << std::endl;
		return firstOffset;
	}
	else {
		//std::cout << "second " << currentScore << std::endl;
		return secondOffset;
	}


}



Offset Hillclimb::climber(Offset prevOffset) {
	resetCheckedOffset();
	stepSize = map->config.TILE_SIZE;
	int count = 0;
	currentOffset = prevOffset;//prevoius offset informative of new offset
	
	currentScore = gpu->getScore(currentOffset);
	Offset newOffset;
	float newScore = 0.0;
	
	while (stepSize > minDist) {
		newOffset = getNextOffset();
		
		if (newOffset.x > 90) {//out of range all directions checked
			decStep();//dec step size
			resetCheckedOffset();//reset checked directions
			//std::cout << "reduce"  << std::endl;
		}
		else {

			newScore = gpu->getScore(newOffset);
		
			
			//std::cout << "score " << newScore << std::endl;
			count++;
			if (newScore < currentScore) {//better spot
				incStep();//inc step size
				resetCheckedOffset();
				currentScore = newScore;
				currentOffset.x = newOffset.x;
				currentOffset.y = newOffset.y;
				currentOffset.theta = newOffset.theta;
				//std::cout << "score " << newScore << std::endl;
				//std::cout << "off X " << currentOffset.x << " off y " << currentOffset.y << " off t " << currentOffset.theta << std::endl;
			}
		}
		

	}
	//std::cout << " count " << count << std::endl;
	return currentOffset;


}



void Hillclimb::decStep() {
	stepSize = stepSize / 2.0;
}

void Hillclimb::incStep() {
	stepSize = stepSize * 2.0;
}

Offset Hillclimb::getNextOffset(){
	Offset offset;
	int direction = random.getRandom(0, 6);
	int inc = 1;//used t ocheck each direction in turn
	while (hasBeenChecked(direction) == true) {//loop until unchecked direction found
		if (inc > 5) {//if all directions checked break at minimum
			offset.x = 99;//make out of range so ignored
			return offset;
		}
		
		direction += inc;
		direction = direction % 6;
		inc++;
	}
	//std::cout << "dir " << direction << std::endl;
	offset.x = currentOffset.x;
	offset.y = currentOffset.y;
	offset.theta = currentOffset.theta;
	if (direction == 0) {
		offset.x += stepSize;
		if (offset.x > 1.0) {
			offset.x = 1.0;
		}
	}
	else if (direction == 1) {
		offset.x -= stepSize;
		if (offset.x < -1.0) {
			offset.x = -1.0;
		}
	}
	else if (direction == 2) {
		offset.y += stepSize;
		if (offset.y > 1.0) {
			offset.y = 1.0;
		}
	}
	else if (direction == 3) {
		offset.y -= stepSize;
		if (offset.y < - 1.0) {
			offset.y = -1.0;
		}
	}
	else if (direction == 4) {
		offset.theta += (stepSize / thetaStepReduction);
		if (offset.theta > 3.14) {
			offset.theta = 3.14;
		}
	}
	else {//(direction == 5) {
		offset.theta -= (stepSize / thetaStepReduction);
		if (offset.theta < -3.14) {
			offset.theta = -3.14;
		}
	}
	return offset;
}

bool Hillclimb::hasBeenChecked(int direction) {
	
	if (offsetChecked[direction] == false) {
		offsetChecked[direction] = true;
		return false;
	}
	else {
		return true;
	}
}

void Hillclimb::resetCheckedOffset() {
	for (int i = 0; i < 6; i++) {
		offsetChecked[i] = false;
	}

}
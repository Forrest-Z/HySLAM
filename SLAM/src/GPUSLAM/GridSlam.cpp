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

#include "GridSlam.h"


 GridSlam::GridSlam(Map* newmap, Data* newdata, Robot* newrobot) {
	map = newmap;
	data = newdata;
	robot = newrobot;
	gpu.setUp(map, data, robot);
}


 void GridSlam::setUp() {
	 gpu.setUp(map, data, robot);
 }

void GridSlam::runNextFrame() {
	//main loop 
	/*
	get raw data
	convert to world cordinats
	claculate sum
	calc distance
	loop with hiliiclimbing algorthm

	*/
	if (map->config.applyCorectionGridSlam == true) {
		//auto start = high_resolution_clock::now();
		Hillclimb hc(map, &gpu);


		Offset bestOffset = hc.climb(lastOffset);//hilcimb
		//auto start2 = high_resolution_clock::now();
		

		Pose RobotPose = robot->getCorrectedPose();
		Pose newPose;
		newPose.x = RobotPose.x + bestOffset.x;
		newPose.y = RobotPose.y + bestOffset.y;
		newPose.theta = RobotPose.theta + bestOffset.theta;

		MiniMap m(map);
		m.createMiniMapNoBlur(data->getFrameWithOffset(bestOffset), newPose);
		//m.createMiniMap(data->getFrameWithOffset(bestOffset), newPose);
		//m.createQuickMiniMap(data->getFrameWithOffset(bestOffset), newPose);
		map->addMiniMap(m.getMap());
		robot->addOffset(bestOffset);

		lastOffset = bestOffset;

		//auto stop = high_resolution_clock::now();
		//auto duration1 = duration_cast<microseconds>(start2 - start);
		//auto duration2 = duration_cast<microseconds>(stop - start);
		//std::cout << "hilclimb Time: " << duration1.count() << std::endl;
		//std::cout << "all Time: " << duration2.count() << std::endl;
	}
	else {//no correction applied  
		MiniMap m(map);
		Pose RobotPose = robot->getCorrectedPose();
		Offset none;
		robot->addOffset(none);//done to ensure log taken
		m.createMiniMapNoBlur(data->getFrame(), RobotPose);
		map->addMiniMap(m.getMap());
		
	}
	
	/*

	Offset testOffset;
	Logger logs;
	float inc = -0.4f;
	float score = 0;
	for (int i = 0; i < 80; i++) {
		inc += 0.01;
		testOffset.x = inc;
		testOffset.y = 0.0;
		testOffset.theta = inc;
		score = gpu.getScore(testOffset);
		std::cout << inc << " " << score << std::endl;
		logs.loginfo(inc, score);
	}

	inc = -0.4f;
	for (int i = 0; i < 80; i++) {
		inc += 0.01;
		testOffset.x = inc;
		testOffset.y = inc;
		testOffset.theta = 0.0;
		score = gpu.getScore(testOffset);
		std::cout << inc << " " << score << std::endl;
		logs.loginfo(inc, score);
	}

	inc = -0.4f;
	for (int i = 0; i < 80; i++) {
		inc += 0.01;
		testOffset.x = 0.0;
		testOffset.y = inc;
		testOffset.theta = inc;
		score = gpu.getScore(testOffset);
		std::cout << inc << " " << score << std::endl;
		logs.loginfo(inc, score);
	}


	*/








}


void GridSlam::cleanUp() {
	gpu.cleanUp();
}

Offset GridSlam::getRecomendedOffset() {
	return lastOffset;
}
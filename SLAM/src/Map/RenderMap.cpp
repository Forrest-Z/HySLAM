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

#include "RenderMap.h"


RenderMap::RenderMap(std::string fileName) {
	this->fileName = fileName;
}

void RenderMap::render(Map& map) {
	fileName += ".pgm";
	std::cout << "render map size: " << map.Count() << std::endl;

	std::ofstream myfile;

	int maxX = map.getMaxX();
	int maxY = map.getMaxY();
	myfile.open(fileName);
	//myfile << "P2 " << map.Xdimention() << " " << map.Ydimention() << " 10\n";
	myfile << "P2 " << map.Xdimention() << " " << map.Ydimention() << " 20\n";


	for (int y = map.getMinY(); y <= maxY; y++) {
		std::string line = "";
		for (int x = map.getMinX(); x <= maxX; x++) {
			
			float p = map.getOccupiedTileProb(x, y);
		    p *= 10;
			p += 10;
			////
			int p1 = (int)p;
			line += " " + std::to_string(p1);

			
		}
		line += "\n";
		//std::cout << line;
		myfile << line;
	}
	myfile.close();


}

void RenderMap::renderRGB(Map& map) {
	fileName += ".ppm";
	std::cout << "render map size: " << map.Count() << std::endl;

	std::ofstream myfile;

	int maxX = map.getMaxX();
	int maxY = map.getMaxY();
	myfile.open(fileName);
	//myfile << "P2 " << map.Xdimention() << " " << map.Ydimention() << " 10\n";
	myfile << "P3 " << map.Xdimention() << " " << map.Ydimention() << " 10\n";


	for (int y = map.getMinY(); y <= maxY; y++) {
		std::string line = "";
		for (int x = map.getMinX(); x <= maxX; x++) {

			std::string p = map.getRGBProb(x, y);
			line += " " + p;

		}
		line += "\n";
		//std::cout << line;
		myfile << line;
	}
	myfile.close();


}
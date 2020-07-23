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

#include "MiniMap.h"


MiniMap::MiniMap(Map* newmap) {
	map = newmap;
	noiseTile = (map->config.NOISE / map->config.TILE_SIZE) + 0.5;
}



void MiniMap::createQuickMiniMap(FrameCoords data, Pose robotPose) {
	miniMap.clear();
	noiseTile = noiseTile - 1;
	std::unordered_map<unsigned int, MMTile> mapofTiles;
	//std::cout << "data size: " << data.points.size() << std::endl;
	for (int i = 0; i < data.points.size(); i++) {
		Point point = data.points[i];
		if (point.x < 9000) {
			int x = map->convertCoords(point.x);
			int y = map->convertCoords(point.y);
			unsigned int ID = map->getID(x, y);
			MMTile tile;
			tile.ID = ID;
			tile.x = x;
			tile.y = y;
			tile.occupied = 1.0;
			mapofTiles[ID] = tile;
			for (int nx = -noiseTile; nx <= noiseTile; nx++) {//blur
				for (int ny = -noiseTile; ny <= noiseTile; ny++) {
					MMTile tile2;
					tile2.x = nx + x;
					tile2.y = ny + y;
					unsigned int id = map->getID(tile2.x, tile2.y);
					tile2.ID = id;
					float prob = 0.0;
					if (std::abs(nx) + std::abs(ny) <= noiseTile) {
						prob = 0.3;
						if ((std::abs(nx) + std::abs(ny) <= noiseTile / 2)) {
							prob = 0.6;
						}
					}
					MMTile* p = &mapofTiles[id];
					
					tile2.occupied = p->occupied;
					tile2.occupied += prob;
					*p = tile2;
				}
			}
			Line line = createLine(robotPose, point);
			setTilesOnLineJustClear(line);
		}
	}
	for (auto x : mapofTiles) {
		// Accessing KEY from element pointed by it.
		//unsigned int key = x.first;
		// Accessing VALUE from element pointed by it.
		MMTile value = x.second;
		if (value.occupied > 1.0) {//ensure not greater than 1
			value.occupied = 1.0;
		}
		miniMap.push_back(value);
		//std::cout << key << " :: " << value << std::endl;

		// Increment the Iterator to point to next entry
		
	}
		

}

void MiniMap::createMiniMapNoBlur(FrameCoords data, Pose robotPose) {
	miniMap.clear();
	//std::cout << "data size: " << data.points.size() << std::endl;
	for (int i = 0; i < data.points.size(); i++) {
		Point point = data.points[i];

		if (point.x < 9000) {
			Line line = createLine(robotPose, point);
			setTilesOnLineNoBlur(line);
		}

	}

}

void MiniMap::createMiniMap(FrameCoords data, Pose robotPose) {
	miniMap.clear();
	//std::cout << "data size: " << data.points.size() << std::endl;
	for (int i = 0; i < data.points.size(); i++) {
		Point point = data.points[i];
		
		if (point.x < 9000) {
			Line line = createLine(robotPose, point);
			setTilesOnLine(line);
		}
		
	}

}

std::vector<MMTile> MiniMap::getMap() {
	//std::cout << "mapsize: " << miniMap.size() << std::endl;
	return miniMap;
}


Line MiniMap::createLine(Pose robot, Point end) {
    Line line;
    Point start;
    start.x = robot.x;
    start.y = robot.y;

    line.a = start;
    line.b = end;

    //line.centre.x = (line.b.x + line.a.x) / 2.0;
    //line.centre.y = (line.b.y + line.a.y) / 2.0;
 
    return line;
}


void MiniMap::setTilesOnLine(Line line) {
	double diffX = line.b.x - line.a.x;
	double diffY = line.b.y - line.a.y;
	if (std::abs(diffX) > std::abs(diffY)) {//then horizontal line
		int noTiles = ((std::abs(diffX) + map->config.NOISE) / map->config.TILE_SIZE) + 1.5;
		double offset = 0.0;
		double ratioChange = diffY / diffX;
		Point currentPoint = line.a;
		
		float probIncrease = 1.0 / (float)noiseTile;
		float currentProb = -1.0;
		for (int i = 0; i < noTiles; i++) {
			//start at robot position
			currentPoint.x = line.a.x + offset;
			currentPoint.y = line.a.y + (offset * ratioChange);
			
			int x = map->convertCoords(currentPoint.x);
			int y = map->convertCoords(currentPoint.y);

			unsigned int ID = map->getID(x,y);
			MMTile tile;
			tile.ID = ID;
			tile.x = x;
			tile.y = y;


			if (i < noTiles - (2 * noiseTile)) {
				//tile.occupied = -1.0;
			}
			else if (i < noTiles - noiseTile) {
				currentProb += probIncrease * 2.0;
			}
			else {
				currentProb -= probIncrease;
			}
		
			tile.occupied = currentProb;

			miniMap.push_back(tile);///should maby change to non list - no duplicuts

			if (diffX < 0) {//line in negative direction
				offset -= map->config.TILE_SIZE;
			}
			else { // line in posative direction
				offset += map->config.TILE_SIZE;
			}
		}
	}
	
	else{////then vertical line
		int noTiles = ((std::abs(diffY) + map->config.NOISE) / map->config.TILE_SIZE) + 1.5;
		double offset = 0.0;
		double ratioChange = diffX / diffY;
		Point currentPoint = line.a;

		float probIncrease = 1.0 / (float)noiseTile;
		float currentProb = -1.0;
		for (int i = 0; i < noTiles; i++) {
			//start at robot position
			currentPoint.y = line.a.y + offset;
			currentPoint.x = line.a.x + (offset * ratioChange);


			int x = map->convertCoords(currentPoint.x);
			int y = map->convertCoords(currentPoint.y);

			unsigned int ID = map->getID(x, y);
			MMTile tile;
			tile.ID = ID;
			tile.x = x;
			tile.y = y;

		
			if (i < noTiles - (2 * noiseTile)) {
				//tile.occupied = -1.0;
			}
			else if (i < noTiles - noiseTile) {
				currentProb += probIncrease * 2.0;
			}
			else {
				currentProb -= probIncrease;
			}

			tile.occupied = currentProb;

			miniMap.push_back(tile);

			if (diffY < 0) {//line in negative direction
				offset -= map->config.TILE_SIZE;
			}
			else { // line in posative direction
				offset += map->config.TILE_SIZE;
			}
		}
	}
	

}


void MiniMap::setTilesOnLineNoBlur(Line line) {
	double diffX = line.b.x - line.a.x;
	double diffY = line.b.y - line.a.y;
	if (std::abs(diffX) > std::abs(diffY)) {//then horizontal line
		int noTiles = (std::abs(diffX) / map->config.TILE_SIZE) + 0.5;;
		double offset = 0.0;
		double ratioChange = diffY / diffX;
		Point currentPoint = line.a;
		float currentProb = -1.0;
		for (int i = 0; i <= noTiles; i++) {
			//start at robot position
			currentPoint.x = line.a.x + offset;
			currentPoint.y = line.a.y + (offset * ratioChange);
			int x = map->convertCoords(currentPoint.x);
			int y = map->convertCoords(currentPoint.y);
			unsigned int ID = map->getID(x, y);
			MMTile tile;
			tile.ID = ID;
			tile.x = x;
			tile.y = y;
			if (i == noTiles) {//last point - laser hit
				currentProb = 1;
			}
			tile.occupied = currentProb;

			miniMap.push_back(tile);///should maby change to non list - no duplicuts

			if (diffX < 0) {//line in negative direction
				offset -= map->config.TILE_SIZE;
			}
			else { // line in posative direction
				offset += map->config.TILE_SIZE;
			}
		}
	}

	else {////then vertical line
		int noTiles =( std::abs(diffY) / map->config.TILE_SIZE) + 0.5;;
		double offset = 0.0;
		double ratioChange = diffX / diffY;
		Point currentPoint = line.a;
		float currentProb = -1.0;
		for (int i = 0; i <= noTiles; i++) {
			//start at robot position
			currentPoint.y = line.a.y + offset;
			currentPoint.x = line.a.x + (offset * ratioChange);
			int x = map->convertCoords(currentPoint.x);
			int y = map->convertCoords(currentPoint.y);
			unsigned int ID = map->getID(x, y);
			MMTile tile;
			tile.ID = ID;
			tile.x = x;
			tile.y = y;
			if (i == noTiles ) {//last point - laser hit
				currentProb = 1;
			}
			tile.occupied = currentProb;
			miniMap.push_back(tile);
			if (diffY < 0) {//line in negative direction
				offset -= map->config.TILE_SIZE;
			}
			else { // line in posative direction
				offset += map->config.TILE_SIZE;
			}
		}
	}


}




void MiniMap::setTilesOnLineJustClear(Line line) {
	double diffX = line.b.x - line.a.x;
	double diffY = line.b.y - line.a.y;
	if (std::abs(diffX) > std::abs(diffY)) {//then horizontal line
		int noTiles = (std::abs(diffX) / map->config.TILE_SIZE) + 0.5;;
		double offset = 0.0;
		double ratioChange = diffY / diffX;
		Point currentPoint = line.a;
		
		for (int i = 0; i <= noTiles - (noiseTile + 1); i++) {
			//start at robot position
			currentPoint.x = line.a.x + offset;
			currentPoint.y = line.a.y + (offset * ratioChange);
			int x = map->convertCoords(currentPoint.x);
			int y = map->convertCoords(currentPoint.y);
			unsigned int ID = map->getID(x, y);
			MMTile tile;
			tile.ID = ID;
			tile.x = x;
			tile.y = y;

			tile.occupied = -1;

			miniMap.push_back(tile);///should maby change to non list - no duplicuts

			if (diffX < 0) {//line in negative direction
				offset -= map->config.TILE_SIZE;
			}
			else { // line in posative direction
				offset += map->config.TILE_SIZE;
			}
		}
	}

	else {////then vertical line
		int noTiles = (std::abs(diffY) / map->config.TILE_SIZE) + 0.5;;
		double offset = 0.0;
		double ratioChange = diffX / diffY;
		Point currentPoint = line.a;
		
		for (int i = 0; i <= noTiles - (noiseTile + 1); i++) {
			//start at robot position
			currentPoint.y = line.a.y + offset;
			currentPoint.x = line.a.x + (offset * ratioChange);
			int x = map->convertCoords(currentPoint.x);
			int y = map->convertCoords(currentPoint.y);
			unsigned int ID = map->getID(x, y);
			MMTile tile;
			tile.ID = ID;
			tile.x = x;
			tile.y = y;
		
			tile.occupied = -1;

			miniMap.push_back(tile);
			if (diffY < 0) {//line in negative direction
				offset -= map->config.TILE_SIZE;
			}
			else { // line in posative direction
				offset += map->config.TILE_SIZE;
			}
		}
	}


}
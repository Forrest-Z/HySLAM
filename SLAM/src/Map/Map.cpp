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

#include "Map.h"


Map::Map() {
	HistoryManager hm(config.HISTORY_FRAMES);
	history = hm;
}

Map::Map(int historyFrames) {
	config.HISTORY_FRAMES = historyFrames;
	HistoryManager hm(config.HISTORY_FRAMES);
	history = hm;
	std::cout << "hist frames: " << config.HISTORY_FRAMES << std::endl;
}

//get the id from coordanats 

unsigned int Map::getID(int x, int y){
	//assume that the max number of unsigend int is 4294967295 -> 4,200,000,000 for simplisity 
	//root of this is 64,807 -> use 64,000 for simplisity - this will allow a max size of 64000 by 64000
	


	//first offset number so all positive:
	int newX = x + 32000;
	int newY = y + 32000;


	unsigned int id = newX * 64000 + newY;
	return id;
}

unsigned int Map::getID(double x, double y) {
	int ix = convertCoords(x);
	int iy = convertCoords(y);
	return getID(ix,iy);
}

int Map::getXCords(unsigned int ID) {
	int x = ID / 64000;
	x -= 32000;
	return x;
}

int Map::getYCords(unsigned int ID) {
	int y = ID % 64000;
	y -= 32000;
	return y;
}

//return probability for given facing - use facing none for occupied prob
float Map::getTileProb(int x, int y, Facing facing) {
	unsigned int ID = getID(x, y);


	if (keyExists(ID) == false){
		return 0.0f;
	}
	else {
		if (facing == Facing::N) {
			return  map[ID].probN;
		}
		else if (facing == Facing::S) {
			return  map[ID].probS;
		}
		else if (facing == Facing::E) {
			return  map[ID].probE;
		}
		else if (facing == Facing::W) {
			return  map[ID].probW;
		}
		else {
			return  map[ID].occupied;
		}
		
	}
}

//will return ocupied prob - faster than using getTileProb when wanting occupied prob
float Map::getOccupiedTileProb(int x, int y) {
	unsigned int ID = getID(x, y);


	if (keyExists(ID) == false) {
		return 0.0f;
	}
	else {
		//std::cout << map[ID].occupied << std::endl;
		return map[ID].occupied;
	}
}


void Map::addMiniMap(std::vector<MMTile> miniMap) {
	//auto start = high_resolution_clock::now();
	for (int i = 0; i < miniMap.size(); i++) {
		addProb(miniMap[i].x, miniMap[i].y, miniMap[i].occupied * config.PROBABILITY_REDUCTION, Facing::None);
	}
	//auto stop = high_resolution_clock::now();
	//auto duration = duration_cast<microseconds>(stop - start);
	//std::cout << "Time: " << duration.count() << std::endl;
}


//returns a rgb string of the probabilitys - used for map render
std::string Map::getRGBProb(int x, int y) {
	unsigned int ID = getID(x, y);


	if (keyExists(ID) == false) {
		return "5 5 5";
	}
	else {
		if (map[ID].probN > 0.0 || map[ID].probW > 0.0 || map[ID].probS > 0.0 || map[ID].probE > 0.0) {
			if (map[ID].probN >= map[ID].probE && map[ID].probN >= map[ID].probS && map[ID].probN >= map[ID].probW) {
				return std::to_string((int)(map[ID].probN * 5) + 5) + " 5 5";
			}
			else if (map[ID].probE >= map[ID].probN && map[ID].probE >= map[ID].probS && map[ID].probE >= map[ID].probW) {
				return "5 " + std::to_string((int)(map[ID].probE * 5) + 5) + " 5";
			}
			else if (map[ID].probS >= map[ID].probE && map[ID].probS >= map[ID].probN && map[ID].probS >= map[ID].probW) {
				return "5 5 " + std::to_string((int)(map[ID].probS * 5) + 5);
			}
			else {//if (map[ID].probW >= map[ID].probE && map[ID].probW >= map[ID].probN && map[ID].probW >= map[ID].probS) {
				return "5 " + std::to_string((int)(map[ID].probW * 5) + 5) + " " + std::to_string((int)(map[ID].probW * 5) + 5);
			}
		}
		//else if (map[ID].occupied > map[ID].probE && map[ID].occupied > map[ID].probS && map[ID].occupied > map[ID].probW && map[ID].occupied > map[ID].probN) {
		else if(map[ID].occupied > 0.1){
			return std::to_string((int)(map[ID].occupied * 5) + 5) + " " + std::to_string((int)(map[ID].occupied * 5) + 5) + " " + std::to_string((int)(map[ID].occupied * 5) + 5);
		}
		else if (map[ID].occupied < -0.1) {//negative space
			return std::to_string((int)(map[ID].occupied * 5) + 5) + " " + std::to_string((int)(map[ID].occupied * 5) + 5) + " " + std::to_string((int)(map[ID].occupied * 5) + 5);
		}
		
		else {
			return "5 5 5";
		}
		
		
	}
}


float Map::getTileProbD(double x, double y, Facing facing) {
	int ix = convertCoords(x);
	int iy = convertCoords(y);

	return getTileProb(ix, iy,facing);
}

double Map::convertCoords(int xy) {
	double cord = (double)xy;
	return cord * config.TILE_SIZE;
}

int Map::convertCoords(double xy) {
	
	xy /= config.TILE_SIZE;
	
	if (xy > 0) {
		xy += 0.5;
	}
	else {
		xy -= 0.5;
	}
	
	int cord = (int)xy;
	return cord;
}

//add probability to map tile - for OrthoSLAM
//Note: also calcualtes nin max map sizes
bool Map::addProb(int x, int y, float prob, Facing facing) {
	float* p;
	unsigned int ID = getID(x, y);

	//update history
	Tile t = map[ID];
	HistoryTile historyTile;
	historyTile.ID = ID;
	historyTile.occupied = t.occupied;
	historyTile.probE = t.probE;
	historyTile.probN = t.probN;
	historyTile.probS = t.probS;
	historyTile.probW = t.probW;

	history.addTile(historyTile);

	if (facing == Facing::N) {
		p = &map[ID].probN;
	}
	else if (facing == Facing::S) {
		p = &map[ID].probS;
	}
	else if (facing == Facing::E) {
		p = &map[ID].probE;
	}
	else if (facing == Facing::W){
		p = &map[ID].probW;
	}
	else {
		p = &map[ID].occupied;
	}
	
	
	if (*p < config.PROBABLITY_THRESHOLD && *p > (-1*config.PROBABLITY_THRESHOLD)) { // only change value if not landmark 
		*p += prob;
	}
	else if (*p > 0 && prob > 0) {
		*p += prob;
	}
	else if (*p < 0 && prob < 0) {
		*p += prob;
	}
	
	
	if (*p > 1.0f) {//cant be greater than 1
		*p = 1.0f;
		
	}
	else if (*p < -1.0f) {//cant be less than -1
		*p = -1.0f;
	}
	
	//now update ocupied prob to the max facing value
	//if (*p > map[getID(x, y)].occupied) {
	//	map[getID(x, y)].occupied = *p;
	//}



	

	//update min and max x and y positions
	if (x > maxX) {
		maxX = x;
	}
	else if (x < minX) {
		minX = x;
	}

	if (y > maxY) {
		maxY = y;
	}
	else if (y < minY) {
		minY = y;
	}
	return true;
}

//called if tile has been seen by OrthoSLAM - increses prob by PROBABLITY_INCREASE
void Map::tileSeen(int x, int y, Facing facing) {
	addProb(x, y, config.PROBABLITY_INCREASE, facing);
}



unsigned int Map::Count() {
	return map.size();
}

int Map::Xdimention() {
	return 1 + maxX - minX;
}

int Map::Ydimention() {
	return 1 + maxY - minY;
}

int Map::getMaxX(){
	return maxX;
}

int Map::getMaxY() {
	return maxY;
}

int Map::getMinX() {
	return minX;
}

int Map::getMinY() {
	return minY;
}

bool Map::keyExists(unsigned int ID) {
	
	if (map.find(ID) == map.end()) {
		return false;
	}
	else {
		return true;
	}
}

//returns true if landmark
bool Map::isLandmark(int x, int y, Facing facing) {
	/*
	float prob = getTileProb(x, y, facing);
	if (prob >= config.PROBABLITY_THRESHOLD) {
		return true;
	}
	else {
		if (prob < 0.01) {
			prob = getTileProb(x, y, Facing::None);
			if (prob > config.PROBABLITY_THRESHOLD) {
				return true;
			}
		}

		return false;
	}

	*/
	if (config.OrthoSLAMWriteToRobotMap == true) {//when writing to map only use facing lines
		float prob = getTileProb(x, y, facing);
		if (prob >= config.PROBABLITY_THRESHOLD) {
			return true;
		}
		else {
			float p1 = prob;
			//if (facing == Facing::N || facing == Facing::S) {
				p1 +=  (getTileProb(x, y - 1, facing) + getTileProb(x, y + 1, facing)) * 0.5 + (getTileProb(x, y - 2, facing) + getTileProb(x, y + 2, facing)) * 0.25;
			//}else{
				p1 += (getTileProb(x-1, y, facing) + getTileProb(x+1, y, facing)) * 0.5  + (getTileProb(x-2, y, facing) + getTileProb(x+ 2,y , facing)) * 0.25;
			//}
			
			if (p1 >= config.PROBABLITY_THRESHOLD) {
				return true;
			}

			return false;
		}
	}
	else {
		unsigned int ID = getID(x, y);

		if (keyExists(ID) == false) {
			return false;
		}
		else {
			float prob = 0.0;
			if (facing == Facing::N) {
				prob = map[ID].probN;
			}
			else if (facing == Facing::S) {
				prob = map[ID].probS;
			}
			else if (facing == Facing::E) {
				prob = map[ID].probE;
			}
			else if (facing == Facing::W) {
				prob = map[ID].probW;
			}
			else {
				prob = map[ID].occupied;
			}

			if (prob >= config.PROBABLITY_THRESHOLD) {
				return true;
			}
			else {

				float prob1 = map[ID].probN;
				float prob2 = map[ID].probE;
				float prob3 = map[ID].probS;
				float prob4 = map[ID].probW;
				if (prob1 <= 0.01 && prob2 <= 0.01 && prob3 <= 0.01 && prob4 <= 0.01) {//all effictivly 0
					prob = map[ID].occupied;

				}
				if (prob >= config.PROBABLITY_THRESHOLD) {
					return true;
				}
				else {
					return false;
				}

			}
		}
	}
	
	


	//float prob = getTileProb(x, y, facing);
	//if (prob >= config.PROBABLITY_THRESHOLD) {
	//	return true;
	//}
	//else {
	//	return false;
	//}
}


void Map::startNextHistoryFrame() {
	history.StartNewFrame();
}


Offset Map::resetMapWithHistory() {
	std::vector<HistoryTile> his = history.getTileHistory();
	std::cout << "roll back" << std::endl;
	for (int i = his.size() - 1; i >= 0; i--) {
		HistoryTile ht = his[i];
		//std::cout << "his tile ID: " << ht.ID << std::endl;
		Tile* t = &map[ht.ID];
		t->occupied = ht.occupied;
		t->probE = ht.probE;
		t->probW = ht.probW;
		t->probS = ht.probS;
		t->probN = ht.probN;
	}
	Offset historyOffset = history.getOffsetHistory();
	history.reset();
	return historyOffset;

}

void Map::resetHistory() {

	history.reset();


}

void Map::addOffsetToHistory(Offset robotOffset) {
	history.setOffset(robotOffset);
}
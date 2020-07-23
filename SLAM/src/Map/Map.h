//absolute map
#pragma once
#include <map>
#include <unordered_map>
#include <vector>
#include "Tile.h"
#include "../ImportData/Constants.h"
#include "HistoryManager.h"

//#include <chrono> //time

//using namespace std::chrono;

class Map
{
	HistoryManager history;
	std::unordered_map<unsigned int, Tile> map;// map stored as sparse list of tiles: each tile has unique ID used as its key based on its coordinate in the grid.
	int minX = 0;//current lower x edge of the map
	int maxX = 0;//current upper x edge of the map
	int minY = 0;//current lower y edge of the map
	int maxY = 0;//current upper y edge of the map

public:
	Config config;
	Map();
	Map(int histFrames);//set up map with number of histroy frames to store - deafult to 7
	bool addProb(int x, int y, float prob, Facing facing);// add/ update tile probabilitys to map - also adds to history
	float getTileProb(int x, int y, Facing facing);// get current tile prob 
	float getOccupiedTileProb(int x, int y);
	float getTileProbD( double x, double y, Facing facing);
	unsigned int Count();//get number of tiles in map
	int Xdimention();
	int Ydimention();
	int getMinX();//return map x lower limit
	int getMaxX();//return map x upper limit
	int getMinY();//return map y lower limit
	int getMaxY();//return map y upper limit
	double convertCoords(int xy);// get the real world coordinate at grid location
	int convertCoords(double xy);//returns a grid coordinate for given real world coordinate, can give an x or y real world coordinate and will retrun corect regardless
	bool isLandmark(int x, int y, Facing facing);//return true if landmark for given facing
	void tileSeen(int x, int y, Facing facing); //adds OrthoSLAM line tiles to map
	std::string getRGBProb(int x, int y);//used for map render to image, returns RGB vaule for given tile

	unsigned int getID(double x, double y);//return ID of tile that is at these real world coordinats
	unsigned int getID(int x, int y);//return ID of tile that is at these grid coordinats

	void addMiniMap(std::vector<MMTile> miniMap);

	void startNextHistoryFrame();
	Offset resetMapWithHistory();
	void addOffsetToHistory(Offset robotOffset);

	void resetHistory();
	
private:
	
	int getXCords(unsigned int ID);
	int getYCords(unsigned int ID);
	bool keyExists(unsigned int ID);

};


//local view map for GPUGM-SLam - uses same ID system as main map
#pragma once
#include "../Map/Map.h"
#include "../Structures.h"
#include "../Map/RenderMap.h"
#include <vector>
#include <unordered_map>
class MiniMap
{
	//std::map<unsigned int, Tile> miniMap;
	std::vector<MMTile> miniMap;
	

	Map* map;

	int noiseTile = 0;//allowed noise converted to tiles - set in config
	

public:
	MiniMap(Map* newmap);

	void createMiniMap(FrameCoords data, Pose robotPose);//old style not used - blured only along lidar lines
	void createQuickMiniMap(FrameCoords data, Pose robotPose);//new blured minimap
	void createMiniMapNoBlur(FrameCoords data, Pose robotPose);//new unblured minimap
	std::vector < MMTile> getMap();
private:
	Line createLine(Pose robot, Point lidar);
	void setTilesOnLine(Line line);
	void setTilesOnLineNoBlur(Line line);
	void setTilesOnLineJustClear(Line line);

	
	
};


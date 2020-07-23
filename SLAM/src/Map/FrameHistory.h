// this class stores a snapshot of a frame before it is added to the absolute map, can the nbe used to rollback map
#pragma once
#include <vector>
#include "Tile.h"
#include "../Structures.h"



class FrameHistory
{
	std::vector<HistoryTile> history;//list of tile in frame
	Offset offset;//offset applied this frame

public: 
	FrameHistory();
	void addTile(HistoryTile tile);//add a tile to this frames history
	void setOffset(Offset offset);//set offset of frame history
	Offset getOffset();//return offset
	std::vector<HistoryTile> getHistory();//return list of tiles in frame
};


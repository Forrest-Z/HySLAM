//manages the history of the map to allow for rollback
#pragma once
#include "FrameHistory.h"
#include<vector>
#include <iostream>

class HistoryManager
{
	std::vector<FrameHistory> history;
	int historySize;//max number of history frames to store
public:
	HistoryManager(int historySize = 1);//set up - sets number of frames to store, defult = 3
	void StartNewFrame();//start next frame in history
	void addTile(HistoryTile tile);//add frame to current frame in history
	void setOffset(Offset offset);//set curent frame offset
	std::vector<HistoryTile> getTileHistory();//return full hisotry for all frames
	Offset getOffsetHistory();//return full offst history
	void reset();// clear history
};


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

#include "HistoryManager.h"


HistoryManager::HistoryManager(int historySize) {
	this->historySize = historySize;
}
void HistoryManager::StartNewFrame() {
	
	FrameHistory fh;
	if (history.size() < historySize) {
		history.push_back(fh);
	}
	else {
		history.erase(history.begin());
		history.push_back(fh);
	}
	std::cout << "history size: " << history.size() << std::endl;
	if (history.size() > historySize) {
		std::cout << "history to big Error" << std::endl;
		throw "history to big!";
	}
}
void HistoryManager::addTile(HistoryTile tile) {
	history[history.size() - 1].addTile(tile);
}
std::vector<HistoryTile> HistoryManager::getTileHistory() {
	std::vector<HistoryTile> allHistory;
	for (int i = 0; i < history.size(); i++)
	{
		FrameHistory fh = history[i];
		std::vector<HistoryTile> nextFrame = fh.getHistory();
		allHistory.insert(allHistory.end(), nextFrame.begin(), nextFrame.end());
	}

	return allHistory;
}


Offset HistoryManager::getOffsetHistory() {
	Offset allHistory;
	allHistory = history[0].getOffset();
	/*
	for (int i = 0; i < history.size(); i++)
	{
		FrameHistory fh = history[i];
		Offset nextFrame = fh.getOffset();
		allHistory.theta += nextFrame.theta;
		allHistory.x += nextFrame.x;
		allHistory.y += nextFrame.y;
	}
	*/
	return allHistory;
}

void HistoryManager::setOffset(Offset offset) {
	history[history.size() - 1].setOffset(offset);
}

void HistoryManager::reset() {
	history.clear();
}
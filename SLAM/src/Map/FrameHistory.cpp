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


#include "FrameHistory.h"
FrameHistory::FrameHistory() {

}
void FrameHistory::addTile(HistoryTile tile) {
	history.push_back(tile);
}
std::vector<HistoryTile> FrameHistory:: getHistory() {
	return history;
}

void FrameHistory::setOffset(Offset offset) {
	this->offset = offset;
}

Offset FrameHistory::getOffset() {
	return offset;
}

/*
    HySLAM: score calulator
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

#include "Align.h"

Align::Align() {

}
std::vector<LinePoint> Align::alignStartPoint(std::vector<LinePoint> groundTruth, std::vector<LinePoint> resultsToAlign) {
	LinePoint gTstart = groundTruth[20];
	LinePoint rStart = resultsToAlign[20];
	std::cout << "raw t: " << resultsToAlign[20].theta << std::endl;
	std::cout << "cor t: " << groundTruth[20].theta << std::endl;
	
	double rotDiff = gTstart.theta - rStart.theta;
	ModifyData md;

	resultsToAlign = md.rotate(rotDiff, resultsToAlign);

	rStart = resultsToAlign[20];
	double xDiff = gTstart.x - rStart.x;
	double yDiff = gTstart.y - rStart.y;
	std::cout << "allign: "<< xDiff << " " << yDiff << " " << rotDiff << std::endl;
	resultsToAlign = md.offset(xDiff, yDiff, resultsToAlign);
	return resultsToAlign;
}


std::vector<LinePoint> Align::alignMidPoint(std::vector<LinePoint> groundTruth, std::vector<LinePoint> resultsToAlign) {
	int i = groundTruth.size() - 50;
	LinePoint gTstart = groundTruth[i];
	LinePoint rStart = resultsToAlign[i];
	std::cout << "raw t: " << resultsToAlign[i].theta << std::endl;
	std::cout << "cor t: " << groundTruth[i].theta << std::endl;

	double rotDiff = gTstart.theta - rStart.theta;
	ModifyData md;

	resultsToAlign = md.rotate(rotDiff, resultsToAlign);

	rStart = resultsToAlign[i];
	double xDiff = gTstart.x - rStart.x;
	double yDiff = gTstart.y - rStart.y;
	std::cout << "allign: " << xDiff << " " << yDiff << " " << rotDiff << std::endl;
	resultsToAlign = md.offset(xDiff, yDiff, resultsToAlign);
	return resultsToAlign;
}

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
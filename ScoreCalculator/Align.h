#pragma once
#include "importFile.h"
#include "ModifyData.h"
class Align
{
public:
	Align();

	std::vector<LinePoint> alignStartPoint(std::vector<LinePoint> groundTruth, std::vector<LinePoint> resultsToAlign);
	std::vector<LinePoint> alignMidPoint(std::vector<LinePoint> groundTruth, std::vector<LinePoint> resultsToAlign);
};


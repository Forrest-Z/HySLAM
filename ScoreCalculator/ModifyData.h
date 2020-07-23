#pragma once
#include "importFile.h"
class ModifyData
{
public:
	ModifyData();
	std::vector<LinePoint> rotate(double radians, std::vector<LinePoint> data);
	std::vector<LinePoint> offset(double x, double y, std::vector<LinePoint> data);
};


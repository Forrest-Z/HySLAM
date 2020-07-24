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
#include "ModifyData.h"

ModifyData::ModifyData() {

}
std::vector<LinePoint> ModifyData::rotate(double radians, std::vector<LinePoint> data) {

	LinePoint center;

	for (int i = 0; i < data.size(); i++) {
		LinePoint point = data[i];
		if (point.type == DataType::Data) {
			double rotatedX = std::cos(radians) * (point.x - center.x) - std::sin(radians) * (point.y - center.y) + center.x;
			double rotatedY = std::sin(radians) * (point.x - center.x) + std::cos(radians) * (point.y - center.y) + center.y;
			data[i].x = rotatedX;
			data[i].y = rotatedY;
			data[i].theta += radians;
		}

	}

	return data;
}




std::vector<LinePoint> ModifyData::offset(double x, double y, std::vector<LinePoint> data) {

	

	for (int i = 0; i < data.size(); i++) {
		LinePoint point = data[i];
		if (point.type == DataType::Data) {
			point.x += x;
			point.y += y;

			data[i] = point;
		}
	}

	return data;
}
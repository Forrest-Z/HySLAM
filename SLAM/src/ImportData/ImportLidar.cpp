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

#include "ImportLidar.h"


ImportLidar::ImportLidar(std::string fileName, char delimeter) {
	this->fileName = fileName;
	this->delimiter = delimeter;
}



std::vector<LDataLine> ImportLidar::getData() {
	// Create an input filestream
	std::ifstream myFile(fileName);
	// Make sure the file is open
	if (!myFile.is_open()) throw std::runtime_error("Could not open file");

	// Helper vars
	std::string line;
	std::cout << "start import"<< std::endl;
	std::vector<LDataLine> result;
	int countline = 0;
	try {
		while (std::getline(myFile, line))
		{
			//std::cout << "import  line: " << countline << std::endl;
			countline++;
			if (line.length() > 1) {
				std::vector<std::string> cont;
				std::stringstream ss(line);
				std::string token;

				while (getline(ss, token, delimiter)) {
					cont.push_back(token);
				}

				LDataLine newData = parseData(cont);
				if (newData.type == dataType::Lidar || newData.type == dataType::Odometry) {
					result.push_back(newData);
				}
			}

		}
	}
	catch (...) {
		std::cout << "failed import at line: " << countline << std:: endl;
		std::cout << "line: " << line << std::endl;
		throw "failed import";
	}

	// Read data, line by line
	

	// Close file
	myFile.close();

	return result;
}

//pase line from input (calls to lidar and odom functions)
LDataLine ImportLidar::parseData(std::vector<std::string> line) {


	std::string name = line[0];

	if (name.compare("FLASER") == 0) {
		return parseLaserData(line);
	}
	else if (name.compare("ODOM") == 0) {
		return parseOdometryData(line);
	}
	else {
		LDataLine newLidarData;
		newLidarData.type = dataType::Unknown;
		return newLidarData;
	}


}

// parse Lidar data 
LDataLine ImportLidar::parseLaserData(std::vector<std::string> line) {
	LDataLine newLidarData;
	newLidarData.name = line[0];
	newLidarData.type = dataType::Lidar;

	//calc number of readings
	int numberOfReadings;
	try
	{
		numberOfReadings = (int)std::stoi(line[1]);
	}
	catch (std::invalid_argument const& e)
	{
		std::cout << "Bad input for numbre of lidar readings: std::invalid_argument thrown" << std::endl;
		throw "error";
	}
	catch (std::out_of_range const& e)
	{
		std::cout << "Integer overflow: std::out_of_range thrown" << std::endl;
		throw "error";
	}

	newLidarData.readings = numberOfReadings;

	//read in lidar data;
	try {
		for (int i = 2; i < numberOfReadings + 2; i++) {
			float val = std::stof(line[i]);
			//std::cout << val << endl;
			newLidarData.lidarReadings.push_back(val);
		}
	}
	catch (std::invalid_argument const& e) {
		std::cout << "invalid input for lidar data" << std::endl;
		throw "error";
	}
	catch (std::out_of_range const& e)
	{
		std::cout << "lidar data conversion" << std::endl;
		throw "error";
	}
		

	
	try {
		newLidarData.x = std::stod(line[numberOfReadings + (int)2]);//convert to float

		newLidarData.y = std::stod(line[numberOfReadings + (int)3]);

		newLidarData.theta = std::stod(line[numberOfReadings + (int)4]);

		
	}
	catch (...) {
		std::cout << "issue with x y theta conversion" << std::endl;
		throw "error";
	}

	try {
		newLidarData.timeStamp = std::stod(line[line.size() - (int)1]);
	}
	catch (...) {
		newLidarData.timeStamp = 0.0;
	}
	

	


	//std::cout << newLine.x << " " << newLine.y << " " << newLine.theta << " " << newLine.timeStamp << endl;
	return newLidarData;
}

//parse odometry data line
LDataLine ImportLidar::parseOdometryData(std::vector<std::string> line) {
	LDataLine newOdomData;
	newOdomData.name = line[0];
	newOdomData.type = dataType::Odometry;



	newOdomData.x = std::stof(line[1]);//convert to float
	newOdomData.y = std::stof(line[2]);
	newOdomData.theta = std::stof(line[3]);
	try {
		newOdomData.timeStamp = std::stof(line[line.size() - 1]);
	}
	catch (...) {
		newOdomData.timeStamp = 0.0;
	}
	

	//std::cout << newLine.x << " " << newLine.y << " " << newLine.theta << " " << newLine.timeStamp << endl;
	return newOdomData;
}



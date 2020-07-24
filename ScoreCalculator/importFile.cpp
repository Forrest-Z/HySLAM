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
#include "importFile.h"

importFile::importFile() {

}
std::vector<LinePoint> importFile::getFile(std::string filename) {
	// Create an input filestream
	filename = "Import/" + filename;
	std::ifstream myFile(filename);

	// Make sure the file is open
	if (!myFile.is_open()) {
		std::cout << "not open file" << std::endl;
		throw std::runtime_error("Could not open file");
	}
	// Helper vars
	std::string line;
	std::cout << "file: " << filename << std::endl;
	std::vector<LinePoint> result;
	// Read data, line by line
	int noline = 0;
	while (std::getline(myFile, line))
	{

		//std::cout << "Line: " << noline << std::endl;
		noline++;
		std::vector<std::string> info;
		line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end()); //remove white space 
		std::stringstream ss(line);
		std::string token;
		while (std::getline(ss, token, ',')) {
			//token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
			info.push_back(token);
		}
		

		if (info[0] == "!roleback") {
			
			int roleback = std::stoi(info[1]);

			for (int i = 0; i < roleback; i++) {
				if (result.size() > 1) {//keep first line not data
					result.pop_back();
				}
				
			}
		}
		else if (info.size() == 10) {//only read line if more than two caracters and does not start with +
			//std::cout << cont[0] << " " << cont[1] << std::endl;

			LinePoint point;
			point.type = DataType::Config;
			point.alpha = std::stod(info[1]);
			point.beta = std::stod(info[3]);
			point.gamma = std::stod(info[5]);
			point.delta = std::stod(info[7]);
			point.minframes = std::stoi(info[9]);
			result.push_back(point);

			//std::cout << point.x << " " << point.y << " " << point.theta << " " << point.SLAM << std::endl;
		}
		else if (info.size() == 29) {//only read line if more than two caracters and does not start with +
			//std::cout << cont[0] << " " << cont[1] << std::endl;
			
			LinePoint point;
			point.type = DataType::Data;
			point.TimeStamp = (info[1]);
			point.x = std::stod(info[6]);
			point.y = std::stod(info[12]);
			point.theta = std::stod(info[18]);
			if (info[2][0] == 'O') {//ortho
				point.SLAM = SlamType::Ortho;
			}
			else {
				point.SLAM = SlamType::GPU;
			}
			point.linesDetected = std::stoi(info[22]);
			point.linesAligned = std::stoi(info[24]);
			point.time = std::stoi(info[28]);
			result.push_back(point);

			//std::cout << point.x << " " << point.y << " " << point.theta << " " << point.SLAM << std::endl;
		}

	}
	std::cout << result.size() << std::endl;
	// Close file
	myFile.close();
	return result;
}

void importFile::printData(std::vector<LinePoint>  results) {
	for (int i = 0; i < results.size(); i++) {
		std::cout << results[i].x << " " << results[i].y << " " << results[i].theta << std::endl;
	}
	
}



std::vector<Importpair> importFile::getFileList() {
	// Create an input filestream
	std::string filename = "Import/FileList.csv";


	std::ifstream myFile(filename);

	// Make sure the file is open
	if (!myFile.is_open()) throw std::runtime_error("Could not open file");

	// Helper vars
	std::string line;

	std::vector<Importpair> result;
	// Read data, line by line
	while (std::getline(myFile, line))
	{
		std::vector<std::string> info;
		line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end()); //remove white space 
		std::stringstream ss(line);
		std::string token;
		while (std::getline(ss, token, ',')) {
			//token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
			info.push_back(token);
		}

		if (info.size() == 2) {
			Importpair ip;
			ip.corrected = info[0];
			ip.raw = info[1];
			result.push_back(ip);
		}

	}
	std::cout << "number of files to score: " << result.size() << std::endl;
	// Close file
	myFile.close();
	return result;
}
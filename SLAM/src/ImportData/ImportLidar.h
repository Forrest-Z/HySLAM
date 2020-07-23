//class imports data files in the Carmen format

#pragma once
#include "LidarData.h"
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream


//using namespace std;

class ImportLidar
{
	std::string fileName;
	char delimiter;
public:
	ImportLidar(std::string fileName, char delimiter = ' ');
	std::vector<LDataLine> getData();

private:
	//std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str);
	LDataLine parseData(std::vector<std::string> line);
	LDataLine parseLaserData(std::vector<std::string> line);
	LDataLine parseOdometryData(std::vector<std::string> line);
};


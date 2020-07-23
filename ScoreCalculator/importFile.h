#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <map>
#include <algorithm>
#include <iomanip>

enum class DataType
{
	Data,
	Config,
	None
};

enum class SlamType {
	GPU,
	Ortho,
	Unknown
};

struct LinePoint {
	DataType type = DataType::None;

	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	SlamType SLAM = SlamType::Unknown;
	int linesDetected = 0;
	int linesAligned = 0;
	int time = 0;
	std::string TimeStamp = "?";


	double alpha = 0;
	double beta = 0;
	double gamma = 0;
	double delta = 0;
	int minframes = 0;

};

struct Importpair {
	std::string raw = "";
	std::string corrected = "";
};

class importFile
{
public:
	importFile();
	std::vector<LinePoint> getFile(std::string file);
	void printData(std::vector<LinePoint>  results);
	std::vector<Importpair> getFileList();


};


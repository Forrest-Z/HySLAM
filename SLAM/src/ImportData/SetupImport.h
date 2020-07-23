//class used for automation for running mutiple runthoughs, see import folder for details
#pragma once


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



struct ImportSet {
	std::string raw = "";
	double alpha = 0;
	double beta = 0;
	double gamma = 0;
	double delta = 0;
	int minFrames = 0;
};

class SetupImport
{
public:
	SetupImport();

	std::vector<ImportSet> getFileList();


};




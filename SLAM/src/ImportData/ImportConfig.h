//Class handles importation of configureation file
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



struct BaseObject
{
	float f = 0.0f;
	std::string s = "";
	int i = 0;
};


const std::string PATH = "config/config.txt";

class ImportConfig
{
public:
	ImportConfig();
	std::map<std::string, BaseObject> import();
private:
	std::map<std::string, BaseObject> parse(std::map<std::string, std::string> conf);
};


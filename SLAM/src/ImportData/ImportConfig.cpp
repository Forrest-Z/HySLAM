#include "ImportConfig.h"

ImportConfig::ImportConfig() {

}

std::map<std::string, BaseObject> ImportConfig::import() {
	// Create an input filestream
	std::ifstream myFile(PATH);
	// Make sure the file is open
	if (!myFile.is_open()) throw std::runtime_error("Could not open file");

	// Helper vars
	std::string line;

	std::map<std::string, std::string> result;

	// Read data, line by line
	while (std::getline(myFile, line))
	{

		line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end()); //remove white space 
		std::vector<std::string> cont;
		std::stringstream ss(line);
		std::string token;
		if (line.size() > 2 && line[0] != '#' ) {//only read line if more than two caracters and does not start with #
			while (getline(ss, token, '=')) {
				//token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
				cont.push_back(token);
			}
			//std::cout << cont[0] << " " << cont[1] << std::endl;
			result[cont[0]] = cont[1];

		}

	}

	std::map<std::string, BaseObject> result2 = parse(result);

	// Close file
	myFile.close();
	return result2;

}

std::map<std::string, BaseObject> ImportConfig::parse(std::map<std::string, std::string> conf) {
	std::map<std::string, BaseObject> result;
	std::string floatKeys[] = { "TILE_SIZE", "PROBABLITY_THRESHOLD", "PROBABLITY_INCREASE", "MIN_OVERLAP"  ,"MAX_ERROR", "MAX_DEVIATION",  "MIN_LINE_LENGTH" , "MAX_LIDAR_RANGE", "NOISE", "PROBABILITY_REDUCTION", "MAX_NOISE" };

	for (int i = 0; i < std::size(floatKeys); i++) {
		BaseObject f1;
		f1.f = std::stof(conf[floatKeys[i]]);
		result[floatKeys[i]] = f1;
	}

	std::string intKeys[] = { "MIN_LINE_POINTS", "LIDAR_COVERAGE" , "MIN_POINTS"};
	for (int i = 0; i < std::size(intKeys); i++) {
		BaseObject i1;
		i1.i = (int)std::stoi(conf[intKeys[i]]);
		result[intKeys[i]] = i1;
	}

	std::string stringKeys[] = { "VERSION" };
	for (int i = 0; i < std::size(stringKeys); i++) {
		BaseObject s1;
		s1.s = (std::string)conf[stringKeys[i]];
		result[stringKeys[i]] = s1;
	}

	return result;
}

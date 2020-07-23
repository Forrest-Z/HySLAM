/*
	Heterogeneous-SLAM: SLAM algorithm
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

#include "SetupImport.h"



SetupImport::SetupImport() {

}






std::vector<ImportSet> SetupImport::getFileList() {
	// Create an input filestream
	std::string filename = "Import/FileList.csv";


	std::ifstream myFile(filename);

	// Make sure the file is open
	if (!myFile.is_open()) throw std::runtime_error("Could not open file");

	// Helper vars
	std::string line;

	std::vector<ImportSet> result;
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

		if (info.size() == 6) {
			ImportSet ip;
			ip.alpha = std::stod(info[0]);
			ip.beta = std::stod(info[1]);
			ip.gamma = std::stod(info[2]);
			ip.delta = std::stod(info[3]);
			ip.minFrames = std::stoi(info[4]);
			ip.raw = info[5];
			
			result.push_back(ip);
		}

	}
	std::cout << result.size() << std::endl;
	// Close file
	myFile.close();
	return result;
}
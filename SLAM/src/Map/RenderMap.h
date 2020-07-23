////clas manages renedering map to image
#pragma once
#include <string>
#include "Map.h"
// basic file operations
#include <iostream>
#include <fstream>


class RenderMap
{
	std::string fileName;
public:
	RenderMap(std::string fileName);//set up render with file name
	void render(Map& map);// render the map in greyscale - does not add orthoslam facing
	void renderRGB(Map& map);// render map in rgb - adds OrthoSLAM facing
};

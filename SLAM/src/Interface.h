//class to make easy to ask question through command line interface
#pragma once
#include <string>
#include <iostream>
class Interface
{
public:
	bool yesNoQuestion(std::string question);
	std::string stringInputQuestion(std::string question);
};


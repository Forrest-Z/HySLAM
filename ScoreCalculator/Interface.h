#pragma once
#include <string>
#include <iostream>
class Interface
{
public:
	bool yesNoQuestion(std::string question);
	std::string stringInputQuestion(std::string question);
};


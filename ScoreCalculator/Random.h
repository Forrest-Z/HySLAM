#pragma once
#include <chrono> //time
#include <iostream>

class Random
{
public:
	Random();

	double getRandom(double min, double max);
	int getRandom(int min, int max);

};


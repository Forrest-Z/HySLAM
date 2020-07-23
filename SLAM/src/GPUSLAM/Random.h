//random class - manages random numbers
#pragma once
#include <chrono> //time
#include <iostream>

class Random
{
public:
	Random();

	double getRandom(double min, double max);//get random double
	int getRandom(int min, int max);//get random int

};


/*
    HySLAM: score calulator
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
#include "Random.h"

Random::Random() {

	//std::srand((unsigned int)time(NULL));
	//std::srand((unsigned int)1111);
	std::srand((unsigned int)2222);
}

double Random::getRandom(double min, double max) {

	double rand = (max - min) * ((double)std::rand() / (double)RAND_MAX) + min;
	//std::cout << "rand: " << rand << std::endl;
	return rand;
}

//get random int - min inclusive, max exclusive
int Random::getRandom(int min, int max) {

	int rand = (std::rand() % max) + min;
	//std::cout << "rand: " << rand << std::endl;
	return rand;
}
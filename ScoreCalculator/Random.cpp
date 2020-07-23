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
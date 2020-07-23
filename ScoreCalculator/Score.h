#pragma once
#include <vector>
#include "importFile.h"
#include "Random.h"
#include "ModifyData.h"
struct Offset {
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
};

class Score
{
	bool offsetChecked[6] = { false,false,false,false, false,false };
	double stepSize = 0.05;
	double currentScore = 999999.9;
	Offset currentOffset;
	Random random;

public:
	Score();
	std::vector<double> calculateScores(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results);
	
	double calculateMeanScore(std::vector<double>);
	void climb(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, std::vector<LinePoint> results2, std::string fileName);
	Offset climber(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results);
private:
	double calcFrameScore(LinePoint groundTruth, LinePoint result);
	void write(std::vector<LinePoint> rawdata, std::vector<LinePoint> cordata, std::vector<double> scores, double score,  double OrthoPersent, std::string fileName);

	double getScore(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, Offset offset);

	void getFinalScore(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, Offset offset, std::string fileName);
	void calculateFinalScores(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, std::string fileName);
	void resetCheckedOffset();
	Offset getNextOffset();
	bool hasBeenChecked(int direction);
	void incStep();
	void decStep();
};


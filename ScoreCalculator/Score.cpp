#include "Score.h"

Score::Score() {

}
std::vector<double> Score::calculateScores(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results) {
	if (groundTruth.size() != results.size()) {
		throw "not same size error";
	}
	std::vector<double> scores;
	for (int i = 5; i < results.size(); i++) {
		if (results[i].type == DataType::Data) {
			double score = calcFrameScore(groundTruth[i], results[i]);
			scores.push_back(score);
		}
		
	}
	
	return scores;
}

void Score::calculateFinalScores(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, std::string fileName) {
	if (groundTruth.size() != results.size()) {
		throw "not same size error";
	}
	std::vector<double> scores;
	int OrthoCount = 0;
	
	for (int i = 5; i < results.size(); i++) {
		if (results[i].type == DataType::Data) {
			double score = calcFrameScore(groundTruth[i], results[i]);
			scores.push_back(score);

			if (results[i].SLAM == SlamType::Ortho) {
				OrthoCount++;
			}

		}
	}
	
	double percent = (double)OrthoCount / (double)(results.size() - 1);
	
	write(results, groundTruth, scores, calculateMeanScore(scores),percent,fileName);
	
}

double Score::calcFrameScore(LinePoint groundTruth, LinePoint result) {
	double xDiff = std::abs(groundTruth.x - result.x);
	double yDiff = std::abs(groundTruth.y - result.y);
	double diff = xDiff * xDiff + yDiff * yDiff;
	return diff;
}

double Score::calculateMeanScore(std::vector<double> scores) {
	double meanscore = 0.0;
	for (int i = 0; i < scores.size(); i++) {
		meanscore += scores[i];
	}
	meanscore = meanscore / scores.size();
	meanscore = std::sqrt(meanscore);
	return meanscore;
}



void Score::write(std::vector<LinePoint> rawdata, std::vector<LinePoint> cordata, std::vector<double> scores,double score,double OrthoPersent, std::string fileName) {

    // std::cout << "count: " << map.Count() << std::endl;
    std::string newfileName = "Output/" + fileName + ".csv";
    std::ofstream myfile;

    myfile.open(newfileName);
    myfile << "#  combined data\n";

	myfile << "mean square error:," + std::to_string(score) + ",mean error,=AVERAGE(H4:H301)," + "OrthoPercent:," + std::to_string(OrthoPersent) + "\n";



	myfile << "timeStamp,raw X,raw y,raw theta,timestamp,truth x,truth y,truth theta,diss squared,diss\n";

	double dissSum = 0.0;

    for (int i = 5; i < rawdata.size(); i++) {
		double diss = std::sqrt(scores[i-5]);
		dissSum += diss;

        std::string line = "";
		line += rawdata[i].TimeStamp + ",";
		line += std::to_string(rawdata[i].x) + "," + std::to_string(rawdata[i].y) + "," + std::to_string(rawdata[i].theta) + ",";
		line += (cordata[i].TimeStamp) + ",";
		line += std::to_string(cordata[i].x) + "," + std::to_string(cordata[i].y) + "," + std::to_string(cordata[i].theta) + ",";
		line += std::to_string(scores[i-5]) + "," + std::to_string(diss);

        line += "\n";
        //std::cout << line;
        myfile << line;
    }
    myfile.close();
	double time = 0;
	for (int i = 0; i < rawdata.size(); i++) {
		if (rawdata[i].time != 0) {
			time += ((double)rawdata[i].time) / 1000000.0;
		}

	}

	std::ofstream myfile2;

	myfile2.open("Output/all.csv",  std::ios_base::app);

	myfile2 << fileName + "," + std::to_string(rawdata[0].alpha) + "," + std::to_string(rawdata[0].beta) + "," + std::to_string(rawdata[0].gamma) + "," + std::to_string(rawdata[0].delta) + "," + std::to_string(rawdata[0].minframes) + "," + std::to_string(OrthoPersent) + "," + std::to_string(score) + "," + std::to_string(dissSum/scores.size()) + "," + std::to_string(time) + "\n";
	myfile.close();



}

double Score::getScore(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, Offset offset) {
	ModifyData md;
	results = md.rotate(offset.theta, results);
	results = md.offset(offset.x, offset.y, results);
	return calculateMeanScore(calculateScores(groundTruth, results));
}


void Score::getFinalScore(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, Offset offset, std::string fileName) {
	ModifyData md;
	results = md.rotate(offset.theta, results);
	results = md.offset(offset.x, offset.y, results);
	
	calculateFinalScores(groundTruth, results, fileName);
	
}
void Score::climb(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results, std::vector<LinePoint> results2, std::string fileName) {
	Offset off1 = climber(groundTruth, results);
	double newScore = getScore(groundTruth, results, off1);

	Offset off2 = climber(groundTruth, results2);
	double newScore2 = getScore(groundTruth, results2, off2);
	
	if (newScore2 > newScore) {
		std::cout << "first" << std::endl;
		getFinalScore(groundTruth, results, off1, fileName);
		std::cout << "off X " << off1.x << " off y " << off1.y << " off t " << off1.theta << std::endl;
		newScore = getScore(groundTruth, results, off1);
		std::cout << "score " << newScore << std::endl;
	}
	else {
		std::cout << "second" << std::endl;
		getFinalScore(groundTruth, results2, off2, fileName);
		std::cout << "off X " << off2.x << " off y " << off2.y << " off t " << off2.theta << std::endl;
		newScore = getScore(groundTruth, results2, off2);
		std::cout << "score " << newScore << std::endl;
		//std::cout << " count " << count << std::endl;
	}
	
}


Offset Score::climber(std::vector<LinePoint> groundTruth, std::vector<LinePoint> results) {
	stepSize = 0.05;
	currentScore = 999999.9;
	Offset o1;
	currentOffset = o1;
	resetCheckedOffset();
	int count = 0;
	//currentOffset = prevOffset;//prevoius offset informative of new offset

	currentScore = getScore(groundTruth,  results, currentOffset);
	std::cout << "currentScore " << currentScore << std::endl;
	Offset newOffset;
	float newScore = 0.0;

	while (stepSize > 0.001) {
		newOffset = getNextOffset();

		if (newOffset.x > 90) {//out of range all directions checked
			decStep();//dec step size
			resetCheckedOffset();//reset checked directions
			//std::cout << "reduce"  << std::endl;
		}
		else {

			newScore = getScore(groundTruth, results, newOffset);


			//std::cout << "score " << newScore << std::endl;
			count++;
			if (newScore < currentScore) {//better spot
				incStep();//inc step size
				resetCheckedOffset();
				currentScore = newScore;
				currentOffset.x = newOffset.x;
				currentOffset.y = newOffset.y;
				currentOffset.theta = newOffset.theta;
				std::cout << "score " << newScore << std::endl;
				std::cout << "off X " << currentOffset.x << " off y " << currentOffset.y << " off t " << currentOffset.theta << std::endl;
			}
		}


	}


	return currentOffset;


}


void Score::resetCheckedOffset() {
	for (int i = 0; i < 6; i++) {
		offsetChecked[i] = false;
	}

}



void Score::decStep() {
	stepSize = stepSize / 2.0;
}

void Score::incStep() {
	stepSize = stepSize * 2.0;
}

Offset Score::getNextOffset() {
	Offset offset;
	int direction = random.getRandom(0, 6);
	int inc = 1;//used t ocheck each direction in turn
	while (hasBeenChecked(direction) == true) {//loop until unchecked direction found
		if (inc > 5) {//if all directions checked break at minimum
			offset.x = 99;//make out of range so ignored
			return offset;
		}

		direction += inc;
		direction = direction % 6;
		inc++;
	}
	//std::cout << "dir " << direction << std::endl;
	offset.x = currentOffset.x;
	offset.y = currentOffset.y;
	offset.theta = currentOffset.theta;
	if (direction == 0) {
		offset.x += stepSize;
		if (offset.x > 1.0) {
			offset.x = 1.0;
		}
	}
	else if (direction == 1) {
		offset.x -= stepSize;
		if (offset.x < -1.0) {
			offset.x = -1.0;
		}
	}
	else if (direction == 2) {
		offset.y += stepSize;
		if (offset.y > 1.0) {
			offset.y = 1.0;
		}
	}
	else if (direction == 3) {
		offset.y -= stepSize;
		if (offset.y < -1.0) {
			offset.y = -1.0;
		}
	}
	else if (direction == 4) {
		offset.theta += (stepSize / 10);
		if (offset.theta > 3.14) {
			offset.theta = 3.14;
		}
	}
	else {//(direction == 5) {
		offset.theta -= (stepSize / 10);
		if (offset.theta < -3.14) {
			offset.theta = -3.14;
		}
	}
	return offset;
}

bool Score::hasBeenChecked(int direction) {

	if (offsetChecked[direction] == false) {
		offsetChecked[direction] = true;
		return false;
	}
	else {
		return true;
	}
}
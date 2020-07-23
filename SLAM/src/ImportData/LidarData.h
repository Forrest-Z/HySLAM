//a holder for one frame of data
#pragma once
#include <vector>
#include <string>
//using namespace std;

enum class dataType { Lidar, Odometry, Unknown };

struct LDataLine {
	std::string name = "";
	dataType type = dataType::Unknown; // type of data - currently if not lidar ignored
	int readings = 0; // number of lidar readings - typicaly 180 or 360
	double timeStamp = 0.0;// timestamp of datareading taken from data set
	std::vector<double> lidarReadings;// list of lidar distance readings
	double x = 0.0;//robot raw odometry pose
	double y = 0.0;//robot raw odometry pose
	double theta = 0.0;//robot raw odometry pose

};


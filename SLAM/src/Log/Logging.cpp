/*
    Heterogeneous-SLAM: SLAM algorithm
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

#include "Logging.h"


Logger::Logger() {

}

void Logger::setLogName(std::string newfileName) {
    fileName = "log/" + newfileName + ".txt";
    std::string filePath = fileName;

    std::string now = "++++++++++++++++++++++++++++++++++++++++++++++++++++++new start+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++new start+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::ofstream myfile;
    myfile.open(filePath, std::ios_base::app);
    myfile << now << '\n';
    myfile.close();
}

void Logger::setRobotPose(Pose rawPose, Pose correctedPose, Offset offset) {

    this->rawPose = rawPose;
    this->correctedPose = correctedPose;
    this->offset = offset;
}

void Logger::log(int linesDetected, int linesAligned, double pScore, std::string frametime) {
    
    std::string filePath = fileName;

    std::string now = "TimeStamp: ," + std::to_string(time);
    now +="," + SLAMrunning; 
    
    now += ",    raw x: ," + std::to_string(rawPose.x) + ", cor x: ," + std::to_string(correctedPose.x) + ", off x: ," + std::to_string(offset.x) + ", raw y: ," + std::to_string(rawPose.y) + ", cor y: ," + std::to_string(correctedPose.y) + ", off y: ," + std::to_string(offset.y) + ", raw theta: ," + std::to_string(rawPose.theta) + ", cor theta: ," + std::to_string(correctedPose.theta) + ", off theta: ," + std::to_string(offset.theta);
    now += ",linesDetected:," + std::to_string(linesDetected) + ",linesAligned:," + std::to_string(linesAligned) + ",PrevFramePScore:," + std::to_string(pScore) + ",frametime:," + (frametime);
    
    std::ofstream myfile;
    myfile.open(filePath, std::ios_base::app);
    myfile << now <<  '\n';
    myfile.close();

    SLAMrunning = "unknown";//reset
    Pose none;
    rawPose = none;
    correctedPose = none;
    Offset offnone;
    offset = offnone;
}

void Logger::setTime(double newTime) {
    time = newTime;
}

void Logger::logString(std::string info) {
    std::string filePath = fileName;

    std::string now = info;
    std::ofstream myfile;

    myfile.open(filePath, std::ios_base::app);
    myfile << now << '\n';
    myfile.close();
}

void Logger::setSLAM(std::string SLAMrunning) {
    this->SLAMrunning = SLAMrunning;
}


void Logger::loginfo(float inc, float score) {
    
    std::string filePath = fileName;

    std::string now = "";
    now +=  std::to_string(score);
    std::ofstream myfile;
    myfile.open(filePath, std::ios_base::app);
    myfile << now << '\n';
    myfile.close();
}
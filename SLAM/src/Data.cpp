
/*
    HySLAM: SLAM algorithm
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

#include "Data.h"

Data::Data(Logger* newlog,Robot* newrobot, Map* newmap) {
    robot = newrobot;
    map = newmap;
    log = newlog;
}

void Data::import() {
    Interface intface;
    std::string input = intface.stringInputQuestion("Input Lidar data file name : ");
    std::cout << "Loading Lidar Data: " << input << " ..." << std::endl;
    ImportLidar import(input);

    data = import.getData();
    std::cout << "data size:" << data.size() << std::endl;
    //importCorrectedData();

    if (intface.yesNoQuestion("calculate offset?") == false) {
        map->config.applyCorectionOrthoSlam = false;
        map->config.applyCorectionGridSlam = false;
        std::cout << "No correction to be applied! \n Do you wish to run only GPU-SLAM or only OrthoSLAM" << std::endl;
        if (intface.yesNoQuestion("Run Only OrthoSLAM?") == true) {
            map->config.runOnlyOrthoSLAM = true;
        }
        else {
            map->config.runOnlyOrthoSLAM = false;
            map->config.OrthoSLAMWriteToRobotMap = false;//stop orthoslam writing to map
        }
    }
    else {
        std::cout << "Correction will be applied!  " << std::endl;
    }
    int start;
    int runs;
    try {
        std::string startFrame = intface.stringInputQuestion("starting frame : ");
        start = (int)std::stoi(startFrame);

        std::string noruns = intface.stringInputQuestion("ending frame (999999 for all) : ");
        runs = (int)std::stoi(noruns);
    }
    catch (...) {//set defult start frames
        start = 0;
        runs = 999999;
    }

    std::cout << "Start frame: " << std::to_string(start) << " End frame: " << std::to_string(runs) << std::endl;

    ///extract file name without extemtion for log file 

    std::stringstream ss(input);
    std::string token;
    std::vector<std::string> name;
    while (std::getline(ss, token, '.')) {
        //token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        name.push_back(token);
    }
    log->setLogName(name[0] + "_" + std::to_string(start) + "_" + std::to_string(runs));



    count = start;
    endframe = runs;
}


void Data::import(std::string fileName, std::string number) {
    
    ImportLidar import("Import/" + fileName);

    data = import.getData();
    std::cout << "data size:" << data.size() << std::endl;
    std::cout << "Correction will be applied!  " << std::endl;

    int start;
    int runs;
    start = 0;
    runs = 999999;
    std::cout << "Start frame: " << std::to_string(start) << " End frame: " << std::to_string(runs) << std::endl;

    ///extract file name without extemtion for log file 

    std::stringstream ss(fileName);
    std::string token;
    std::vector<std::string> name;
    while (std::getline(ss, token, '.')) {
        //token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        name.push_back(token);
    }
    log->setLogName(name[0] + "_" + number );



    count = start;
    endframe = runs;
}


//return current frame in world coorrdanates
FrameCoords Data::getFrame() {
    return currentFrame;
}

void Data::historyRollBack() {
   // std::string name = "frameHistory/hisCount" + std::to_string(count);
    //RenderMap rm(name);
    //rm.renderRGB(*map);
    log->logString("!roleback," + std::to_string(map->config.HISTORY_FRAMES));
    for (int i = 0; i < map->config.HISTORY_FRAMES + 1; i++) {//loop back frames + 1
        dataType type = dataType::Unknown;
        while (type != dataType::Lidar) {
            
            if (count > 0) {
                count--;
                type = data[count].type;
                if (type == dataType::Lidar) {
                    rawDataCurrentFrame = data[count];
                }
            }
            else {
                count = 0;
                setNextFrame();
                break;
                ///start of data catch - do nothing use first frame
               // throw "end of data";
            }

        }
    }
    count++;//move count to next frame as with set next frame method 

    robot->setTime(rawDataCurrentFrame.timeStamp);//role back raw position
    robot->setRawPose(rawDataCurrentFrame.x, rawDataCurrentFrame.y, rawDataCurrentFrame.theta);
    //clean data

    rawDataCurrentFrame.lidarReadings = noiseRemoval(rawDataCurrentFrame.lidarReadings);

    //now convert data to world coords
    currentFrame = convertToWorldCoords(rawDataCurrentFrame);

    Offset historyOffset = map->resetMapWithHistory();//reset map to mach data
    robot->historyRollBack(historyOffset);//role back offset
   //std::string name2 = name + "b";
   // RenderMap rm2(name2);
    //rm2.renderRGB(*map);
}

//get next frame of raw data and convert to word coorrdanates 
void Data::setNextFrame() {


   // std::string name2 = "count" + std::to_string(test);
    //RenderMap rm2(name2);
    //rm2.renderRGB(*map);
    //test++;


    dataType type = dataType::Unknown;
    while (type != dataType::Lidar) {
        
        if (count < data.size() && count < endframe) {
            rawDataCurrentFrame = data[count];
            type = rawDataCurrentFrame.type;
        }
        else {
            ///end of data catch
            throw "end of data";
        }
        count++;
    }
    //first update robot raw position and timestamp
    robot->setTime(rawDataCurrentFrame.timeStamp);
    robot->setRawPose(rawDataCurrentFrame.x, rawDataCurrentFrame.y, rawDataCurrentFrame.theta);
    //clean data

    rawDataCurrentFrame.lidarReadings = noiseRemoval(rawDataCurrentFrame.lidarReadings);

    //now convert data to world coords
    currentFrame =  convertToWorldCoords(rawDataCurrentFrame);
}

FrameCoords Data::convertToWorldCoords(LDataLine rawData){
    FrameCoords fc;
    double convert = 0.01745329252;// this is pi/180 used for conversion - harcoded for ease
    std::vector<double> lidar = rawData.lidarReadings;
    double angle = (((double)(map->config.LIDAR_COVERAGE)) / lidar.size()) * convert;
    
    
   

    Pose robotPose = robot->getCorrectedPose();
    
    
    double x = 0.0;
    double y = 0.0;
    for (int i = 0; i < lidar.size(); i++) {
        if (lidar[i] < map->config.MAX_LIDAR_RANGE) {
            x = (std::cos((-1.5707963268 + (i * angle)) + robotPose.theta) * lidar[i]) + robotPose.x;
            y = (std::sin((-1.5707963268 + (i * angle)) + robotPose.theta) * lidar[i]) + robotPose.y;
            Point point;
            point.x = x;
            point.y = y;

           // std::cout << "point: " << i << " x: " << x << " y: " << y << std::endl;

            fc.points.push_back(point);
        }
        else {/// point out of lidar effective range - give dummy data to ensure not used in mapping, these points are not discarded to help the line detection
            Point point;
            point.x = 9999;
            point.y = 9999;
            fc.points.push_back(point);
        }

    }



    return fc;
}


FrameCoords Data::getFrameWithOffset(Offset offset) {
    return getFrameWithOffset(offset, map->config.MAX_LIDAR_RANGE);
}


FrameCoords Data::getFrameWithOffset(Offset offset, float maxRange) {
    FrameCoords fc;
    double convert = 0.01745329252;// this is pi/180 used for conversion - harcoded for ease
    std::vector<double> lidar = rawDataCurrentFrame.lidarReadings;
    //removeNoise(lidar);
    double angle = (((double)(map->config.LIDAR_COVERAGE)) / lidar.size()) * convert;

    Pose testRobotPose = robot->getCorrectedPose();
    testRobotPose.x += offset.x;
    testRobotPose.y += offset.y;
    testRobotPose.theta += offset.theta;


    double x = 0.0;
    double y = 0.0;
    for (int i = 0; i < lidar.size(); i++) {
        if (lidar[i] < maxRange) {
            x = (std::cos((-1.5707963268 + (i * angle)) + testRobotPose.theta) * lidar[i]) + testRobotPose.x;
            y = (std::sin((-1.5707963268 + (i * angle)) + testRobotPose.theta) * lidar[i]) + testRobotPose.y;
            Point point;
            point.x = x;
            point.y = y;

            // std::cout << "point: " << i << " x: " << x << " y: " << y << std::endl;

            fc.points.push_back(point);
        }
        else {/// point out of lidar effective range - give dummy data to ensure not used in mapping, these points are not discarded to help the line detection
            Point point;
            point.x = 9999;
            point.y = 9999;
            fc.points.push_back(point);
        }

    }



    return fc;
}

std::vector<double> Data::noiseRemoval(std::vector<double> lidar) {
    if (lidar.size() > 50) {// only reduce noise if enugh lidar data
        int count = 0;
        int start = 0;

        for (int i = 0; i < lidar.size() - 1; i++) {
            double l1 = lidar[i];
            double l2 = lidar[i + 1];

            double diff = std::abs(l1 - l2);
            if (diff < map->config.MAX_NOISE * l1) {
                count++;
            }
            else {
                if (count > map->config.MIN_POINTS) {//keep points as count greter than min
                    count = 0; // reset to 0
                    start = i + 1;//set start to next index
                }
                else {//count to small remove points
                    for (int j = start; j <= i; j++) {
                        lidar[j] = map->config.MAX_LIDAR_RANGE + 1;//set to greater than max diss so will be removed
                        //std::cout << "noise removal: " << j <<  std::endl;
                    }
                    count = 0; // reset to 0
                    start = i + 1;//set start to next index
                }
            }
        }
        return lidar;
    }
    else {
        return lidar;
    }
    
}

void Data::importCorrectedData() {
    Interface intface;
    
    if (intface.yesNoQuestion("Do you want to add corrected data") == true) {
        std::string input = intface.stringInputQuestion("Input corrected Lidar data file name: ");
        std::cout << "Loading corrected Lidar Data: " << input << " ..." << std::endl;

        ImportLidar import(input);

        //data = import.getData();
        correctedData = import.getData();
        combineCorrectedData();

        std::string name = intface.stringInputQuestion("Save Combined data, enter file name: ");
        write(name);
    }

}

void Data::combineCorrectedData() {
    int CorDataPos = 0;
    int DataPos = 0;
    bool found = false;
    std::vector<LDataLine> combinedData;
    while (DataPos < data.size() && CorDataPos < correctedData.size()) {
        //std::cout << "1pos: " << DataPos << " corPos: " << CorDataPos << std::endl;
        while (data[DataPos].type != dataType::Lidar) {
            DataPos++;
        }
        found = false;
        std::cout << "2pos: " << DataPos << " corPos: " << CorDataPos << std::endl;
        while (found == false) {
            while (correctedData[CorDataPos].type != dataType::Lidar && correctedData[CorDataPos].type != dataType::Odometry) {
                CorDataPos++;
            }
            std::cout << "3pos: " << DataPos << " corPos: " << CorDataPos << std::endl;
            if (data[DataPos].timeStamp < correctedData[CorDataPos].timeStamp + 0.0009 && data[DataPos].timeStamp > correctedData[CorDataPos].timeStamp - 0.0009) {
                data[DataPos].x = correctedData[CorDataPos].x;
                data[DataPos].y = correctedData[CorDataPos].y;
                data[DataPos].theta = correctedData[CorDataPos].theta;
                combinedData.push_back(data[DataPos]);
                DataPos++;
                CorDataPos++;
                found = true;
                //CorDataPos == DataPos;
            }
            else {
                CorDataPos++;
                //DataPos--;//this is done to prevent datapos from moving forawd untill match is found
            }

            if (CorDataPos == correctedData.size()) {
                CorDataPos = 0;
            }
        }
        
        
    }



    data = combinedData;
    
}



void Data::write(std::string fileName) {

   // std::cout << "count: " << map.Count() << std::endl;

    std::ofstream myfile;

    myfile.open(fileName);
    myfile << "#  combined data\n";



    for (int i = 0; i < data.size(); i++) {
        std::string line = "";
        line += "FLASER ";
        line += std::to_string(data[i].readings);
        line += " ";
        for (int j = 0; j < data[i].readings; j++) {
            line += std::to_string(data[i].lidarReadings[j]);
            line += " ";
        }
        line += std::to_string(data[i].x);
        line += " ";
        line += std::to_string(data[i].y);
        line += " ";
        line += std::to_string(data[i].theta);
        line += " ";
        line += std::to_string(data[i].x);
        line += " ";
        line += std::to_string(data[i].y);
        line += " ";
        line += std::to_string(data[i].theta);
        line += " ";
        line += std::to_string(data[i].timeStamp);
        line += " nohost ";
        line += std::to_string(data[i].timeStamp);

        line += "\n";
        //std::cout << line;
        myfile << line;
    }
    myfile.close();


}
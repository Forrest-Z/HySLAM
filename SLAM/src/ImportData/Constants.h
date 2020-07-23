//class holds constants used thought SLAM algorithm


#pragma once
#include<string>

#include "ImportConfig.h"

struct Config{
    float TILE_SIZE, PROBABLITY_THRESHOLD, PROBABLITY_INCREASE;
    float MAX_LIDAR_RANGE; // max lidar reading to be considered - any reading further thsan this is igonerd.
    int LIDAR_COVERAGE; // generaly 180 or 360
    float MAX_NOISE = 0.1f; //  the maximum differance in ajencent lidar readings per meter before being removed
    int MIN_POINTS = 5; // minimum number of points to be within max disstance for the points to be kept

    int HISTORY_FRAMES = 7;//the number of frames to wind back upon switching


    float MAX_ERROR; // if a landmark is within 5.0 cm of another landmark its the same landmark
    float MIN_OVERLAP; // min amount of overlap required for lines to be consisered the same
    float MAX_DEVIATION; // max allowed deviation between endpoints for line to be considered orthogonal
    float MIN_LINE_LENGTH;
    int MIN_LINE_POINTS;


    float NOISE = 0.1f; // amount of noise/blur for gpu slam;
    float PROBABILITY_REDUCTION = 0.2f; //   reduciton appliyed to the minimap when added to map.


    std::string VERSION;//program version

    //these are set by user on start up
    bool applyCorectionOrthoSlam = true;//allow orthoSLAM to calculate offsets for robot corrected pose
    bool applyCorectionGridSlam = true;//allow GPUGM-SLAM to apply offsets to robot corrected pose
    bool OrthoSLAMWriteToRobotMap = true;//allow orthoSLAM to apply offsets to robot corrected pose
    bool runOnlyOrthoSLAM = true; // used only when applycorrection  == false


    Config() {
        
        ImportConfig IC;
        std::map<std::string, BaseObject> info = IC.import();

        TILE_SIZE = info["TILE_SIZE"].f;
        PROBABLITY_THRESHOLD = info["PROBABLITY_THRESHOLD"].f;
        PROBABLITY_INCREASE = info["PROBABLITY_INCREASE"].f;
        MAX_ERROR = info["MAX_ERROR"].f;
        MIN_OVERLAP = info["MIN_OVERLAP"].f;
        MAX_DEVIATION = info["MAX_DEVIATION"].f;
        MIN_LINE_LENGTH = info["MIN_LINE_LENGTH"].f;
        MAX_LIDAR_RANGE = info["MAX_LIDAR_RANGE"].f;
        NOISE = info["NOISE"].f;
        PROBABILITY_REDUCTION = info["PROBABILITY_REDUCTION"].f;
        MAX_NOISE = info["MAX_NOISE"].f;


        VERSION = info["VERSION"].s;


        MIN_LINE_POINTS = info["MIN_LINE_POINTS"].i;
        LIDAR_COVERAGE = info["LIDAR_COVERAGE"].i;
        MIN_POINTS = info["MIN_POINTS"].i;

        //print out 
        std::cout << "Config: " <<  std::endl;

        std::cout << "VERSION: " << VERSION << std::endl;
        std::cout << "TILE_SIZE: " << TILE_SIZE << std::endl;
        std::cout << "MAX_LIDAR_RANGE: " << MAX_LIDAR_RANGE << std::endl;
        std::cout << "LIDAR_COVERAGE: " << LIDAR_COVERAGE << std::endl;
        std::cout << "MAX_NOISE: " << MAX_NOISE << std::endl;
        std::cout << "MIN_POINTS: " << MIN_POINTS << std::endl;
       

        std::cout << "OrthoSLAM: " << std::endl;
        std::cout << "PROBABLITY_INCREASE: " << PROBABLITY_INCREASE << std::endl;
        std::cout << "MAX_ERROR: " << MAX_ERROR << std::endl;
        std::cout << "MIN_OVERLAP: " << MIN_OVERLAP << std::endl;
        std::cout << "MAX_DEVIATION: " << MAX_DEVIATION << std::endl;
        std::cout << "MIN_LINE_LENGTH: " << MIN_LINE_LENGTH << std::endl;
        std::cout << "MIN_LINE_POINTS: " << MIN_LINE_POINTS << std::endl;
        std::cout << "PROBABLITY_THRESHOLD: " << PROBABLITY_THRESHOLD << std::endl;

        std::cout << "GPU_SLAM: " << std::endl;
        std::cout << "NOISE: " << NOISE << std::endl;
        std::cout << "PROBABILITY_REDUCTION: " << PROBABILITY_REDUCTION << std::endl;
        std::cout << "  " << std::endl;
      
    }
};


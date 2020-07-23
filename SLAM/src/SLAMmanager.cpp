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

#include "SLAMmanager.h"

SLAMmanager::SLAMmanager(Logger* newlog, Map* newmap, Data* newdata, Robot* newrobot, GridSlam* newGridSlam, OrthoSLAM* newOrthoSlam) {
	map = newmap;
	data = newdata;
	robot = newrobot;
	gridSlam = newGridSlam;
	orthoSlam = newOrthoSlam;
    log = newlog;
}

void SLAMmanager::setup(double newalpha, double newbeta, double newgamma, double newdelta, int newminFrames) {
    alpha = newalpha;
    beta = newbeta;
    gamma = newgamma;
    delta = newdelta;
    minFrames = newminFrames;
}


void SLAMmanager::run() {
    int i = 0;
    int GPUmincounter = 0;
    //data->setNextFrame();
        // orthoSlam->runNextFrame();
        //gridSlam->runStartFrame();
    std::string info = "alpha:," + std::to_string(alpha) + ",beta:," + std::to_string(beta) + ",gamma:," + std::to_string(gamma) + ",delta:," + std::to_string(delta) + ",minframes:," + std::to_string(minFrames);
    log->logString(info);

    map->startNextHistoryFrame();
    data->setNextFrame();
    orthoSlam->runNextFrame();//right angle ortho
    data->setNextFrame();
    orthoSlam->runNextFrame();//right angle ortho
    map->resetHistory();//setup for start
    while (true) {
        //std::string name = "frameHistory/frame" + std::to_string(i);
        //RenderMap rm(name);
        //rm.renderRGB(*map);
        map->startNextHistoryFrame();
        try {
            auto start = high_resolution_clock::now();


            data->setNextFrame();
            if (orthoSlamRunning == true) {
                log->setSLAM("Ortho");
                orthoSlam->runNextFrame();

                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                std::string time = std::to_string(duration.count());
                log->log(orthoSlam->getLinesDetectedCount(), orthoSlam->getLinesAlignedCount(), pScore, time);

                runGPUslamNextFrame(orthoSlam->getLinesDetectedCount(), orthoSlam->getLinesAlignedCount(), orthoSlam->getFrameOffset());
                //gridSlam.runNextFrame();
                std::cout << "Ortho"<< std::endl;
                
                GPUmincounter = 0;
            }
            else {
                log->setSLAM("GPU");
                orthoSlam->runNextFrame();//wont make offset changes as orthoslam running == false
                gridSlam->runNextFrame();

                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                std::string time = std::to_string(duration.count());
                log->log(orthoSlam->getLinesDetectedCount(), orthoSlam->getLinesAlignedCount(), pScore, time);

                if (GPUmincounter > 20) {
                    runGPUslamNextFrame(orthoSlam->getLinesDetectedCount(), orthoSlam->getLinesAlignedCount(), orthoSlam->getFrameOffset(), gridSlam->getRecomendedOffset());
                }
                
                
                std::cout << "GPU" << std::endl;
                
                GPUmincounter++;
            }




            std::cout << "at: " << i << std::endl;
            i++;
        }
        catch (...) {
            std::cout << "ended at: " << i << std::endl;
            break;
        }

    }
}

bool SLAMmanager::runGPUslamNextFrame(int linesDetected, int linesAligned, Offset OrthoOffset) {
    Offset GPU;
    return runGPUslamNextFrame(linesDetected,  linesAligned,  OrthoOffset, GPU);
}
bool SLAMmanager::runGPUslamNextFrame(int linesDetected, int linesAligned, Offset OrthoOffset, Offset GPUoffset) {
    if (map->config.applyCorectionGridSlam == true && map->config.applyCorectionOrthoSlam == true) {
        //if (orthoSlamRunning == true) {
            std::cout << "alinesDetected: " << linesDetected << " linesAligned: " << linesAligned << " OrthoOffset: " << OrthoOffset.x << ", " << OrthoOffset.y << ", " << OrthoOffset.theta << " GPUoffset: " << GPUoffset.x << ", " << GPUoffset.y << ", " << GPUoffset.theta << std::endl;

        //}
        double O = std::abs(OrthoOffset.x - GPUoffset.x) + std::abs(OrthoOffset.y - GPUoffset.y) + std::abs(OrthoOffset.theta - GPUoffset.theta);

        std::cout << "O: " << O << std::endl;
        std::cout << "gamma: " << gamma << std::endl;
        std::cout << "beta: " << beta << std::endl;
       // double P = alpha * (1 / (linesDetected + 1)) + beta * O + gamma * (((linesDetected  - linesAligned) + 1) / (linesDetected + 1));//ad line length?

        pScore =  (beta * O) + (gamma * ((double)(((double)linesDetected - (double)linesAligned) + (double)1) / (double)((double)linesDetected + (double)1)));//ad line length?
       
        std::cout << "P: " << pScore << std::endl;
        if (orthoSlamRunning == true) {
            if (pScore > delta) {
                frameCount++;
            }
            else {
                frameCount--;
            }
        }
        else {//gpu slam running
            if (pScore < (delta / 1.2) ) {
                frameCount++;
            }
            else {
                frameCount--;
            }
        }
        if (frameCount < 0) {
            frameCount = 0;
        }
        //if(true){
        if (frameCount >= (minFrames)  && orthoSlamRunning == true) {//stop running orthoslam - now gpu slam
            frameCount = 0;
            orthoSlamRunning = false;
            map->config.OrthoSLAMWriteToRobotMap = false;//stop orthoslam writing to map
            //gridSlam->setUp();
            //data->historyRollBack();
            return true;
        }
       // else if(true){
        else if (frameCount >= (int)(minFrames  * 1.2)  && orthoSlamRunning == false) {//start running OrthoSLAM - stop gpu slam
            frameCount = 0;
            orthoSlamRunning = true;
            //gridSlam->cleanUp();
            map->config.OrthoSLAMWriteToRobotMap = true;//start orthoslam writing to map
            data->historyRollBack();
            //orthoSlam->addSwitchLockPoint();
            return false;
        }
        else {
            return false;
        }
    }
    else {
        if (map->config.runOnlyOrthoSLAM == false) {
            orthoSlamRunning = false;
            return true;
        }
        else{
            orthoSlamRunning = true;
            return false;
        }
       
    }

}
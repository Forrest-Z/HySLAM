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

#include "LineDetection.h"



LineDetection::LineDetection() {
    data = NULL;
    robot = NULL;
    map = NULL;
}


void LineDetection::setup(Data* newdata, Robot* newrobot, Map* newmap) {
    data = newdata;
    robot = newrobot;
    map = newmap;

    maxDisBetweenPoints = ((map->config.MAX_LIDAR_RANGE * 3.14) / 180) * 1.5;
  
}

//process next data line extract lines
std::vector<Line> LineDetection::getNextFrame() {
	FrameCoords anewData = data->getFrame();
    
	//now extract lines
    std::vector<Line> alines = extractLines(anewData);
    //std::cout << "lines detected: " << alines.size() << std::endl;
	return alines;
    

 

  
}

//process next data line extract lines
std::vector<Line> LineDetection::getStartFrame() {
    FrameCoords anewData = data->getFrame();

    //now extract lines
    std::vector<Line> alines = extractStartLines(anewData);
    //std::cout << "lines detected: " << alines.size() << std::endl;
    return alines;

  
}

std::vector<Line> LineDetection::extractLines(FrameCoords newdata) {
	//use line extraction algrthm to get lines from data
	std::vector<Line> lines;
    int start = 0;
    int end = 1;
    Direction currentDirection = Direction::None;
    Direction nextDirection = Direction::None;
    Point sp;
    Point ep;
    Point nextEp;
    int size = (newdata.points.size()) - 1;
    while (start < size) {// -1 so stops when start = 179 (for 180 readings)
        sp = newdata.points[start];
        ep = newdata.points[end];
        nextEp = ep;
        currentDirection = isOrthogonal(sp, ep, start, end);
        if (currentDirection != Direction::None) {
            nextDirection = currentDirection;
           
            
            if (currentDirection == Direction::Unkown) {
                while (nextDirection == Direction::Unkown && end < size) {//leav loop only if direction is known
                    ep = nextEp;
                    end++;
                    nextEp = newdata.points[end];
                    nextDirection = isOrthogonal(sp, nextEp, start, end);
                }
                //now check previous points are on line
                if (nextDirection != Direction::None) {//if direction notnone and is also known
                    int pointCounter = 0;
                    for (int i = start + 1; i < end; i++) {
                        Point PrevP = newdata.points[i];
                        if (isInLine(sp, PrevP, nextDirection) == true) {//check all intermediat points to see if in line
                            pointCounter++;
                        }
                    }
                    if (pointCounter >= end - (start + 1)) {//if all inline
                        currentDirection = nextDirection;
                    }
                }
               
            }
            
            while (currentDirection == nextDirection && end < size) {

                ep = nextEp;
                end++;
                nextEp = newdata.points[end];
                nextDirection = isOrthogonal(ep, nextEp, end - 1, end);
            }
            
           

           

            //now check possible line
            if (((end - start) > map->config.MIN_LINE_POINTS)) {
                if (currentDirection == Direction::Horizontal && std::abs(sp.x - ep.x) > (double)map->config.MIN_LINE_LENGTH) {

                    //is line

                    lines.push_back(createLine(sp, ep, Direction::Horizontal));
                    
                }
                else if (currentDirection == Direction::Vertical && std::abs(sp.y - ep.y) > (double)map->config.MIN_LINE_LENGTH) {
                    //is line
                    
                    lines.push_back(createLine(sp, ep, Direction::Vertical));
                    
                }
                
            }

            if ((end - start) > 1) {//go back and start from end of last line - diff must be greater than one to prevent infanate loop where cant find line adds one to end then subtracts one .......
               end--;
            }
        }
        start = end;
        end++;
 
    }

    
	return lines; 
}

Direction LineDetection::isOrthogonal(Point start, Point end, int startIndex, int endIndex) {
    double xDiff = std::abs(start.x - end.x);
    double yDiff = std::abs(start.y - end.y);
    int indexDiff = endIndex - startIndex;

    if (start.x > 9000 || end.x > 9000) {//catch removed points
        return Direction::None;
    }

    //if (xDiff < minDisBetweenPoints && yDiff < minDisBetweenPoints) {//curently disable this
   //     return Direction::Unkown;
   // }
    if (xDiff < yDiff)//could be vertical
    {

        if (xDiff < (double)map->config.MAX_DEVIATION * yDiff  && (maxDisBetweenPoints * indexDiff) > yDiff) {//is the x veriation less then that allowed per meter and is the distance between points less than the max disstance
            return Direction::Vertical;
        }
    }
    else //could be horizontal
    {
        if (yDiff < (double)map->config.MAX_DEVIATION * xDiff && (maxDisBetweenPoints * indexDiff) > xDiff) {
            return Direction::Horizontal;
        }
    }

    return Direction::None;
}

//used to check if possible that two points could be in line
bool LineDetection::isInLine(Point start, Point check, Direction direction) {
    double xDiff = std::abs(start.x - check.x);
    double yDiff = std::abs(start.y - check.y);

    if (direction == Direction::Horizontal) {
        if (yDiff < (double)map->config.MAX_DEVIATION * xDiff) {
            return true;
        }
    }
    else if (direction == Direction::Vertical) {
        if (xDiff < (double)map->config.MAX_DEVIATION * yDiff) {
            return true;
        }
    }
    
    return false;

}

Line LineDetection::createLine(Point start, Point end, Direction dir) {
    Line line;

    if (dir == Direction::Horizontal) {
        if (start.x < end.x) {//a must be the left point
            line.a = start;
            line.b = end;
        }
        else {
            line.a = end;
            line.b = start;
        }
        
    }
    else {//vertical
        if (start.y < end.y) {//a must be the lowest point
            line.a = start;
            line.b = end;
        }
        else {
            line.a = end;
            line.b = start;
        }

    }

    line.centre.x = (line.b.x + line.a.x) /2.0;
    line.centre.y = (line.b.y + line.a.y) /2.0;
    line.direction = dir;

    if (line.direction == Direction::Vertical) {
        if (line.centre.x < robot->getCorrectedPose().x) {
            line.facing = Facing::E;
        }
        else {
            line.facing = Facing::W;
        }
    }
    else {
        if (line.centre.y < robot->getCorrectedPose().y) {
            line.facing = Facing::N;
        }
        else {
            line.facing = Facing::S;
        }
    }
    //std::cout << "lines ax: " << line.a.x << " lines ay: " << line.a.y << " lines bx: " << line.b.x << " lines by: " << line.b.y << std::endl;
    return line;
}


Direction LineDetection::isStartLine(Point start, Point end, int startIndex, int endIndex) {
    double xDiff = std::abs(start.x - end.x);
    double yDiff = std::abs(start.y - end.y);

    if (start.x > 9000 || end.x > 9000) {//catch removed points
        return Direction::None;
    }

 

    if (xDiff < yDiff)//could be vertical
    {

        if (xDiff < (double)0.8  * yDiff && maxDisBetweenPoints > yDiff) { // 0.8 allows upto 38 degree angle
            return Direction::Vertical;
        }
    }
    else //could be horizontal
    {
        if (yDiff < (double) 0.8 * xDiff && maxDisBetweenPoints > xDiff) {
            return Direction::Horizontal;
        }
    }

    return Direction::None;
}

std::vector<Line> LineDetection::extractStartLines(FrameCoords newdata) {
    //use line extraction algrthm to get lines from data
    std::vector<Line> lines;
    int start = 0;
    int end = 1;
    Direction currentDirection = Direction::None;
    Direction nextDirection = Direction::None;
    Point sp;
    Point ep;
    Point nextEp;
    int size = (newdata.points.size()) - 1;
    while (start < size) {// -1 so stops when start = 179 (for 180 readings)
        sp = newdata.points[start];
        ep = newdata.points[end];
        nextEp = ep;
        currentDirection = isStartLine(sp, ep, start, end);
        if (currentDirection != Direction::None) {
            nextDirection = currentDirection;

            while (currentDirection == nextDirection && end < size) {

                ep = nextEp;
                end++;
                nextEp = newdata.points[end];
                nextDirection = isStartLine(ep, nextEp, end - 1, end);
            }

            //now check possible line
            if (((end - start) > map->config.MIN_LINE_POINTS)) {
                if (currentDirection == Direction::Horizontal && std::abs(sp.x - ep.x) > (double)map->config.MIN_LINE_LENGTH) {

                    //is line

                    lines.push_back(createLine(sp, ep, Direction::Horizontal));
                }
                else if (currentDirection == Direction::Vertical && std::abs(sp.y - ep.y) > (double)map->config.MIN_LINE_LENGTH) {
                    //is line

                    lines.push_back(createLine(sp, ep, Direction::Vertical));
                }
            }
        }
        start = end;
        end++;

    }


    return lines;
}
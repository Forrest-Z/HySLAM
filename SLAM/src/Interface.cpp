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

#include "Interface.h"

bool Interface::yesNoQuestion(std::string question) {
    std::cout << question << " (y,n)" << std::endl;

    std::string answer = "";
    std::cin >> answer;

    size_t found = answer.find("y");
    if (found != std::string::npos) {
        return true;
    }
    found = answer.find("Y");
    if (found != std::string::npos) {
        return true;
    }
    return false;
}



std::string Interface::stringInputQuestion(std::string question) {
    std::cout << question  ;

    std::string answer = "";
    std::cin >> answer;

    return answer;
}
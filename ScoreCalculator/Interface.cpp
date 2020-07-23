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
    std::cout << question;

    std::string answer = "";
    std::cin >> answer;

    return answer;
}
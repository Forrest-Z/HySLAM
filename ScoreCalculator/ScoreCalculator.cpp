// ScoreCalculator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "importFile.h"
#include "ModifyData.h"
#include "Score.h"
#include "Align.h"
#include "Interface.h"

int main()
{
    std::cout << "The Error Calculator!" << std::endl;
    importFile IF;


    std::vector<Importpair> files = IF.getFileList();

    for (int i = 0; i < files.size(); i++) {

        ModifyData md;
        Score SCR;
        Align align;
        Interface intface;

        //std::string rawname = intface.stringInputQuestion("Enter raw log data file name: ");
        //std::string corname = intface.stringInputQuestion("Enter corected log data file name: ");

        std::string rawname = files[i].raw;
        std::string corname = files[i].corrected;

        //extract ouput file name
        std::stringstream ss(rawname);
        std::string token;
        std::vector<std::string> name;
        while (std::getline(ss, token, '.')) {
            //token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            name.push_back(token);
        }

        std::string fileName = "";
        for (int i = 0; i < name.size() - 1; i++) {
            fileName += name[i] + ".";
        }



        std::vector<LinePoint> groundTruth = IF.getFile(corname);
        std::vector<LinePoint> result = IF.getFile(rawname);

        if (groundTruth.size() != result.size()) {
            std::cout << "wrong size" << std::endl;
            throw "error wrong size";
        }
        else {
            for (int i = 0; i < result.size(); i++) {
                if (result[i].TimeStamp != groundTruth[i].TimeStamp) {
                    std::cout << "doint match: " << i << std::endl;
                    throw "error";
                }
            }
        }

        std::vector<LinePoint> result2 = align.alignStartPoint(groundTruth, result);
        std::vector<LinePoint> result1 = align.alignMidPoint(groundTruth, result);
        SCR.climb(groundTruth, result, result2, fileName);
    }
    //result = md.offset(0, 1, result);
   // std::vector<double> scores = SCR.calculateScores(groundTruth, result);
   // double score = SCR.calculateMeanScore(scores);
    //std::cout << score << std::endl;

    //IF.printData(result);
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

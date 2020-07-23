
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




#include <iostream>
#include "src/Map/Map.h"
#include "src/ImportData/ImportLidar.h"
#include "src/Map/RenderMap.h"
#include "src/ImportData/Constants.h"
#include "src/OrthoSLAM/OrthoSLAM.h"
#include "src/Robot/Robot.h"
#include "src/Interface.h"
#include "src/GPUSLAM/GridSlam.h"
#include "src/SLAMmanager.h"
#include "src/Log/Logging.h"
#include "src/ImportData/SetupImport.h"


int main()

   {
	
  
       

       std::cout << R"(
    HySLAM Copyright (C) 2020 Samuel Haley
    This program comes with ABSOLUTELY NO WARRANTY.
    This is free software, and you are welcome to redistribute it
    under certain conditions; see license.txt for details.

 __  __     __  __     ______     __         ______     __    __    
/\ \_\ \   /\ \_\ \   /\  ___\   /\ \       /\  __ \   /\ "-./  \   
\ \  __ \  \ \____ \  \ \___  \  \ \ \____  \ \  __ \  \ \ \-./\ \  
 \ \_\ \_\  \/\_____\  \/\_____\  \ \_____\  \ \_\ \_\  \ \_\ \ \_\ 
  \/_/\/_/   \/_____/   \/_____/   \/_____/   \/_/\/_/   \/_/  \/_/ 
           -... -.--    ... .- --    .... .- .-.. . -.--         )" << std::endl;
       std::cout << "Hello Welcome to HySLAM!" << std::endl;
       //setup
       Interface intface;

       if (intface.yesNoQuestion("Do you want to run manualy? ") == true) {
           std::cout << "loading" << std::endl;

           Logger log;
           
           //std::cout << "Robot loaded." << std::endl;
           Map map;
           Robot robot(&log, &map);
           //std::cout << "Map created." << std::endl;



           Data data(&log, &robot, &map);
           std::cout << "Data class created." << std::endl;
           OrthoSLAM OS(&map, &data, &robot);
           std::cout << "OrthoSLAM loaded." << std::endl;
           GridSlam gpu(&map, &data, &robot);
           std::cout << "GPU_SLAM loaded." << std::endl;
           SLAMmanager manager(&log, &map, &data, &robot, &gpu, &OS);
           //import data
           try {
               data.import();
           }
           catch (...) {
               std::exit(1); // 
           }

           if (intface.yesNoQuestion("Do you want to start? ") == false) {
               std::cout << "Goodbye!" << std::endl;
           }
           else {
               ///////main loop
               manager.run();


               std::cout << "set up render... " << std::endl;

               std::cout << "Enter save file name (should have .ppm): ";

               std::string saveFile = "";

               std::cin >> saveFile;

               RenderMap renderer(saveFile);

               if (intface.yesNoQuestion("do you want RGB map? ") == true) {
                   std::cout << "RGB saving... " << std::endl;
                   renderer.renderRGB(map);
               }
               else {
                   std::cout << "Greyscale saving... " << std::endl;
                   renderer.render(map);
               }




               gpu.cleanUp();

           }

       }
       else {
           SetupImport setupImp;
           std::vector<ImportSet> values = setupImp.getFileList();
           for (int i = 0; i < values.size(); i++) {
               Logger log;
               
               //std::cout << "Robot loaded." << std::endl;
               Map map(values[i].minFrames + 2);
               Robot robot(&log, &map);
               //std::cout << "Map created." << std::endl;



               Data data(&log, &robot, &map);
               std::cout << "Data class created." << std::endl;
               OrthoSLAM OS(&map, &data, &robot);
               std::cout << "OrthoSLAM loaded." << std::endl;
               GridSlam gpu(&map, &data, &robot);
               std::cout << "GPU_SLAM loaded." << std::endl;
               SLAMmanager manager(&log, &map, &data, &robot, &gpu, &OS);
               manager.setup(values[i].alpha, values[i].beta, values[i].gamma, values[i].delta, values[i].minFrames);
               //import data
               std::string append = std::to_string(values[i].alpha) + "_" + std::to_string(values[i].beta) + "_" + std::to_string(values[i].gamma) + "_" + std::to_string(values[i].delta) + "_" + std::to_string(values[i].minFrames);
               try {
                   data.import(values[i].raw, append);
               }
               catch (...) {
                   continue; // move to next dataset
               }


                ///////main loop
                manager.run();



                std::cout << "set up render... " << std::endl;

                std::stringstream ss(values[i].raw);
                std::string token;
                std::vector<std::string> name;
                while (std::getline(ss, token, '.')) {
                    //token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
                    name.push_back(token);
                }
                std::string saveFile = "log/" + name[0] + "_" + append;

                

               

                RenderMap renderer(saveFile);

                std::cout << "RGB saving... " << std::endl;
                renderer.renderRGB(map);
                




                gpu.cleanUp();

               
           }
       }

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
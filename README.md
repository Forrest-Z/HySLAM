HySLAM is a dynamic scheduling, SLAM algorithm. Designed to run on heterogeneous hardware, HySLAM combines two existing SLAM algorithms, dynamically switching between the two at runtime. These two algorithms, one developed for prioritising low power usage running on a CPU (OrthoSLAM (Nguyen, 2006)) and the other prioritising high accuracy running on a GPU (GPU Grid Matching SLAM, (Rodriguez-Losada, 2013)), allow HySLAM to achieve high accuracy while maintaining low energy usage when operating in indoor environments.

HySLAM has been evaluated. From this evaluation it was found that HySLAM can achieve an RMSE of 0.03m.

This repository contains two programs:
 - The HySLAM.
 - The Scoring Program -  used to calculate the RMSE from the ground truth.

HySLAM builds on work by Nguyen, 2006 and Rodriguez-Losada, 2013. 

HySLAM has two ways of operating:
1.	Manually enter the dataset required in the command line interface.
2.	Set up a list of files to process, in one go. For this the “FileList.csv” is used in the import folder. This allows the weights of the SLAM algorithm to be set.

The Score programme takes two log files from the SLAM program and compare them, returning an RMSE (root mean square error). To import the files, modify the “FileList.csv” file in the import folder.

This project was part of a thesis for Heriot Watt University 

## Datasets

Has been designed to run on Radish datasets: 
Robotics 2D-Laser Datasets: http://www.ipb.uni-bonn.de/datasets/


## Dependencies

HySLAM requires a C++17 compliant compiler. It relies on the OpenCL library 3.3.1 or greater


## Bibliography

Nguyen, V., Harati, A., Martinelli, A., Siegwart, R., and Tomatis, N. (2006). Orthogonal SLAM: A step toward lightweight indoor autonomous navigation. IEEE International Conference on Intelligent Robots and Systems, pages 5007–5012.

Rodriguez-Losada, D., San Segundo, P., Hernando, M., De La Puente, P., and ValeroGomez, A. (2013). GPU-mapping: Robotic map building with graphical multiprocessors. IEEE Robotics and Automation Magazine, 20(2):40–51.

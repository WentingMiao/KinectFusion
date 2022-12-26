#include <iostream>
#include <fstream>
#include <array>

#include "VirtualSensor.h"
#include "Eigen.h"

int main(){
    //  Path of the data folder
	std::string filenameIn = "../Data/rgbd_dataset_freiburg1_xyz/";

    std::cout << "Initialize virtual sensor..." << std::endl;

    VirtualSensor sensor;

    if(!sensor.init(filenameIn)){
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
    }

    
    return 0;
}
#include <iostream>
#include <fstream>
#include <array>


#include "VirtualSensor.h"
#include "Eigen.h"
#include "Frame.h"


int execute(){

    //path to the data 
    std::string filenameIn =  "../Data/rgbd_dataset_freiburg1_xyz/";

    //path to the output 
    std::string filenameBaseOut = "./results/mesh_";
    std::cout << "Initialize virtual sensor..." << std::endl;

    VirtualSensor sensor;

    if(!sensor.Init(filenameIn)){
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
    }
    
    sensor.ProcessNextFrame();

    Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
    Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
    Matrix4f trajectory = sensor.GetTrajectory();

    BYTE* colorMap = &sensor.GetColorRGBX()[0];
    float* depthMap = &sensor.GetDepth()[0];

    unsigned int width  = sensor.GetDepthImageWidth();
    unsigned int height = sensor.GetDepthImageHeight();
    float edgeThreshold = 10;


    Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold);

    /** check if the constructor works **/
    // std::cout<< sensor.GetDepth()[27876]<<std::endl;
    // std::cout<< currentFrame.getDepthMap()[27876]<<std::endl;

    std::vector<Vertex> vertices = currentFrame.getVertices();
    
    
    return 0;
}


int main(){
    
    int res;
    
    res = execute();

    // Vector4f v1 = Vector4f(4.0,2.0,7.0,1.0);
    // Vector4f v2 = Vector4f(3.0,5.0,4.0,1.0);
    // Vector3f v1_ = v1.head<3>();
    // Vector3f v2_ = v2.head<3>();
    // Vector3f normal = v1_.cross(v2_);
    // std::cout<<normal<<std::endl;
    // std::cout<<normal.normalized()<<std::endl;

    return 0;
}
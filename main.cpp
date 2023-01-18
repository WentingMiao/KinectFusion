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
    std::string filenameBaseOut = "../results/mesh_";
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

    vector<float> depthVectorMap = currentFrame.getDepthMap();
    unsigned int levelSize = 5;
    vector<vector<float>> depthPyramid ;

    currentFrame.buildDepthPyramid(depthVectorMap, depthPyramid, levelSize);
    for(int i=0;i<depthPyramid.size();i++){
            cout<<i<<" "<< depthPyramid[i].size() <<endl;
    }

    vector<Vertex> vertices = currentFrame.getVertices();
    
    
    /*write to the mesh*/
    stringstream ss;
	ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
    cout<<ss.str()<<endl;
    if (!currentFrame.writeMesh(vertices,ss.str())){
			cout << "Failed to write mesh!\nCheck file path!" << endl;
			return -1;
	}
    
    
    return 0;
}


int main(){
    
    int res;
    
    res = execute();

    return res;
}
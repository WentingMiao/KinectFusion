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
    
    while(sensor.ProcessNextFrame()){
    
        Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        Matrix4f trajectory = sensor.GetTrajectory();

        BYTE* colorMap = &sensor.GetColorRGBX()[0];
        float* depthMap = &sensor.GetDepth()[0];

        unsigned int width  = sensor.GetDepthImageWidth();
        unsigned int height = sensor.GetDepthImageHeight();
        float edgeThreshold = 10;
        bool filtered = false;


        unsigned int levelSize = 3;
        Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered, levelSize);

        // vector<float> depthVectorMap = currentFrame.getDepthMap();
    
        // vector<vector<float>> depthPyramid ;

        
        // currentFrame.buildDepthPyramid(depthVectorMap, depthPyramid, levelSize);


        // vector<Vertex> vertices = currentFrame.getVertices();
        
        vector<vector<Vertex>> pyramidDepthMap = currentFrame.getPyramidVertex();
        
            

        /*write to the mesh*/
        // stringstream ss;
        // ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << "_LL1"<<".off";
        // cout<<ss.str()<<endl;
        // if (!currentFrame.writeMesh(pyramidDepthMap[1],ss.str(),1)){
        // 		cout << "Failed to write mesh!\nCheck file path!" << endl;
        // 		return -1;
        // }


        for(unsigned int level = 0; level < levelSize; level++){
            stringstream ss;
            ss << filenameBaseOut << sensor.GetCurrentFrameCnt() <<"Level"<< level<< "_test"<<".off";
            cout<<ss.str()<<endl;
            if (!currentFrame.writeMesh(pyramidDepthMap[level],ss.str(),level)){
                    cout << "Failed to write mesh!\nCheck file path!" << endl;
                    return -1;
            }
        }
        



    }
    

    return 0;
}


int main(){
    
    int res;
    
    res = execute();

    return res;
}
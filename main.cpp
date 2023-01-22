
#include <iostream>
#include <fstream>
#include <array>

#include "VirtualSensor.h"
#include "Eigen.h"
#include "Frame.h"
#include "Pose_estimation.h"

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
    bool filtered = true;


    Eigen::Matrix4f cur_pose = Matrix4f::Identity();

    const float distance_threshold = 0.8f;
    const float angle_threshold = 60.0f;
    const int num_iteration = 1;
    const int pyramid_level = 3;

    Frame previousFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered);

    vector<float> depthVectorMap = previousFrame.getDepthMap();

    vector<vector<float>> depthPyramid;
    // level 0 is large

    previousFrame.buildDepthPyramid(depthVectorMap, depthPyramid, pyramid_level);

    vector<Vertex> vertices = previousFrame.getVertices();
    
    
    /*write to the mesh*/
    stringstream ss;
    ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
    cout<<ss.str()<<endl;
    if (!previousFrame.writeMesh(vertices,ss.str())){
            cout << "Failed to write mesh!\nCheck file path!" << endl;
            return -1;
    }
    // Initialization completed (frame 0 finished)

    // frame 1 start
    while(sensor.ProcessNextFrame() && sensor.GetCurrentFrameCnt() <= 1){
        Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        Matrix4f trajectory = sensor.GetTrajectory();

        BYTE* colorMap = &sensor.GetColorRGBX()[0];
        float* depthMap = &sensor.GetDepth()[0];

        unsigned int width  = sensor.GetDepthImageWidth();
        unsigned int height = sensor.GetDepthImageHeight();
        
        Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered);   
        currentFrame.buildDepthPyramid(depthVectorMap, depthPyramid, pyramid_level);

        Pose pose;
        pose.pose_estimation(currentFrame.getVertices(),
                             previousFrame.getVertices(),
                             depthIntrinsics,
                             distance_threshold,
                             angle_threshold,
                             num_iteration,
                             width,
                             height,
                             pyramid_level,
                             cur_pose);            
    
        //get source vertex map (frame k)
        vector<Vertex> vertices = currentFrame.getVertices(); 

        for(auto it = vertices.begin(); it != vertices.end(); ++it){
            it->position = pose.Vector3fToVector4f(pose.TransformToVertex(pose.Vector4fToVector3f(it->position),cur_pose));
        }        
        
        /*write to the mesh*/
        stringstream ss;
        ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
        cout<<ss.str()<<endl;
        if (!currentFrame.writeMesh(vertices,ss.str())){
                cout << "Failed to write mesh!\nCheck file path!" << endl;
                return -1;
        }

        previousFrame = currentFrame;

    }
    
    return 0;   
}


int main(){
    
    int res;
    res = execute();

    

    return res;
}

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


    Frame previousFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold);

    /** check if the constructor works **/
    // std::cout<< sensor.GetDepth()[27876]<<std::endl;
    // std::cout<< currentFrame.getDepthMap()[27876]<<std::endl;

    vector<Vertex> vertices = previousFrame.getVertices();

    
    stringstream ss;
	ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
    cout << ss.str() << endl;
    if (!previousFrame.writeMesh(vertices,ss.str())){
			cout << "Failed to write mesh!\nCheck file path!" << endl;
			return -1;
    }


    sensor.ProcessNextFrame();
    Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold);
    vertices = currentFrame.getVertices();
    stringstream ss_1;
	ss_1 << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
    cout << ss_1.str() << endl;
    if (!previousFrame.writeMesh(vertices,ss_1.str())){
			cout << "Failed to write mesh!\nCheck file path!" << endl;
			return -1;
    }    
    
    // std::cout << previousFrame.getHeight() << std::endl;

    Eigen::Matrix4f cur_pose = Matrix4f::Identity();
    const float distance_threshold = 0.8f;
    const float angle_threshold = 60.0f;
    const int num_iteration = 5;
    const int pyramid_level = 5;
    //TODO all about Initialization see above


    // Initialization completed

    // for (unsigned int i = 1; i < 2; ++i){
    //     std::cout << "Current is Frame: " << i << std::endl;
    //     sensor.ProcessNextFrame();
    //     Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold);

    //     Pose pose;
    //     pose.pose_estimation(currentFrame.getVertices(),
    //                          previousFrame.getVertices(),
    //                          depthIntrinsics,
    //                          distance_threshold,
    //                          angle_threshold,
    //                          num_iteration,
    //                          width,
    //                          height,
    //                          pyramid_level,
    //                          cur_pose);

    //     previousFrame = currentFrame;
    //     vertices = previousFrame.getVertices();
    //     for(auto it = vertices.begin(); it != vertices.end(); ++it){
    //         it->position = pose.Vector3fToVector4f(pose.TransformToVertex(pose.Vector4fToVector3f(it->position),cur_pose));
    //     }
    //     stringstream ss;
    //     ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
    //     cout << ss.str() << endl;        
    //     if (!currentFrame.writeMesh(vertices,ss.str())){
    //     cout << "Failed to write mesh!\nCheck file path!" << endl;
    //     return -1;
    //     }       
    // }
	


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
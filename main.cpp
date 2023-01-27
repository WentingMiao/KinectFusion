#include <iostream>
#include <fstream>
#include <array>

#include "VirtualSensor.h"
#include "Eigen.h"
#include "Frame.h"
#include "Voxels.h"
#include "Pose_estimation.h"

#define USE_ICP 1
// 1 if use icp to optimize pose

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


    /*
        configuration of pose estimation
    */
    Eigen::Matrix4f cur_pose = Matrix4f::Identity();

    const float distance_threshold = 0.01f;
    const float angle_threshold = 20.0f;
    std::vector<int> num_iterations = std::vector<int>{ 10, 5, 4 }; // from last to front
    const int max_level = 3;

    Frame previousFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered, max_level);

    vector<float> depthVectorMap = previousFrame.getDepthMap();

    vector<vector<float>> depthPyramid;
    // level 0 is large

    previousFrame.buildDepthPyramid(depthVectorMap, depthPyramid, max_level);

    vector<Vertex> vertices = previousFrame.getVertices(USE_ICP);
    
    
    /*write to the mesh*/
    stringstream ss;
    ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
    cout<<ss.str()<<endl;
    if (!previousFrame.writeMesh(vertices,ss.str(),0)){
            cout << "Failed to write mesh!\nCheck file path!" << endl;
            return -1;
    }
    // Initialization completed (frame 0 finished)

    // frame 1 start
    while(sensor.ProcessNextFrame() && sensor.GetCurrentFrameCnt() <= 2){
        Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        Matrix4f trajectory = sensor.GetTrajectory();

        BYTE* colorMap = &sensor.GetColorRGBX()[0];
        float* depthMap = &sensor.GetDepth()[0];

        unsigned int width  = sensor.GetDepthImageWidth();
        unsigned int height = sensor.GetDepthImageHeight();
        
        Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered, max_level);   

        // currentFrame.buildDepthPyramid(depthVectorMap, depthPyramid, pyramid_level);

        Pose pose;
        auto level_intrinsics = currentFrame.getLevelCameraIntrinstics();
        auto level_current_vertices = currentFrame.getPyramidVertex(USE_ICP);
        auto level_previous_vertices = previousFrame.getPyramidVertex(USE_ICP);
        auto level_width = currentFrame.getLevelWidth();
        auto level_height = currentFrame.getLevelHeight();
        
        pose.pose_estimation(level_current_vertices,
                             level_previous_vertices,
                             level_intrinsics,
                             distance_threshold,
                             angle_threshold,
                             num_iterations,
                             level_width,
                             level_height,
                             max_level,
                             cur_pose);            
    
        //get source vertex map (frame k)
        vector<Vertex> vertices = currentFrame.getVertices(USE_ICP); 

        for(auto it = vertices.begin(); it != vertices.end(); ++it){
            it->position = pose.Vector3fToVector4f(pose.TransformToVertex(pose.Vector4fToVector3f(it->position),cur_pose));
        }        
	    
	//surface reconstruction
	VoxelArray volume(std::array<unsigned, 3>{600, 600, 600},0.05, Vector3f{ -3, -3, 0 }, Matrix4f{});
        
	Fusion fusion;
	fusion.SurfaceReconstruction(currentFrame, volume, cur_pose, truncationDistance);

        /*write to the mesh*/
        stringstream ss;
        ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
        cout<<ss.str()<<endl;
        if (!currentFrame.writeMesh(vertices,ss.str(),0)){
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

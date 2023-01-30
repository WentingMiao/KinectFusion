#include <iostream>
#include <fstream>
#include <array>

#include "VirtualSensor.h"
#include "Eigen.h"
#include "Frame.h"
#include "Voxels.h"
#include "Pose_estimation.h"
#include "SurfaceReconstruction.hpp"
#include "RayCasting.h"

#define USE_ICP 1
// 1 if use icp to optimize pose

/*
Suggestions from Shuze
0. 传参适当少来点 const T& (穿不了rvalue)
1. 减少while循环里定义的类对象
    （比如frame中不会变的尺寸等参数传入构造函数；一个函数负责接收图片产生vertice pyramid）
    （比如无需保存变量的Fusion类可以变成Fusion namespace）
2. 需要更多封装以令各模块简洁
    理想：
    int main() {
        init();
        while (condition) {
            projection();
            pose estimation();
            volume integration();
            ray casting();
        }
        return 0;
    }
3. 合并冗余函数
    例如转换Camera与World coordinate的函数（可以独立成类专门保存内外参、负责转换）
4. 避免局部变量保存配置参数
    (用宏#DEFINE 比局部变量更清晰、更易于更改)
*/
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

    // vector<Vertex> vertices = previousFrame.getVertices(USE_ICP);
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
        Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered, max_level);   

        // currentFrame.buildDepthPyramid(depthVectorMap, depthPyramid, pyramid_level);

        Pose pose;
        auto level_intrinsics = currentFrame.getLevelCameraIntrinstics();
        auto level_current_vertices = currentFrame.getPyramidVertex(USE_ICP);
        auto level_previous_vertices = previousFrame.getPyramidVertex(USE_ICP);
        previousFrame = currentFrame;
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

        for(auto it = vertices.begin(); it != vertices.end(); ++it)
            if(it->position.x() != MINF)
                it->position = pose.Vector3fToVector4f(pose.TransformToVertex(pose.Vector4fToVector3f(it->position),cur_pose));
	    
	    //surface reconstruction
	    VoxelArray volume(std::array<unsigned, 3>{100, 100, 50}, 0.5, Vector3f{-3, -3, 0}, cur_pose);

        float truncationDistance = 2;
        clock_t begin = clock();
	    Fusion::SurfaceReconstruction(currentFrame, volume, truncationDistance);
        clock_t end = clock();
        double duration = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "SurfaceReconstruction finished in " << duration << " secs" << std::endl;

        
        // ray casting
        RayCasting cast{width, height, cur_pose, volume};
        begin = clock();
        auto [depth, rgbd] = cast.SurfacePrediction();
        end = clock();
        duration = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "ray casting finish in " << duration << " secs" << std::endl;
        // saved in a more global variable so that it can be transformed into pyramid in the next loop
        // auto depth = std::move(std::get<0>(imgs));
        // auto rgbd = std::move(std::get<1>(imgs));

        // testing only begin
        FreeImageB rgbdimg{width, height};
        rgbdimg.data = std::move(rgbd).get();
        rgbdimg.SaveImageToFile("../results/rgbd" + std::to_string(sensor.GetCurrentFrameCnt()) + ".png");
        FreeImage depthimg{width, height, 1};
        depthimg.data = std::move(depth).get();
        depthimg.SaveImageToFile("../results/depth" + std::to_string(sensor.GetCurrentFrameCnt()) + ".png");
        // testing only end

        /*write to the mesh*/
        stringstream ss;
        ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
        cout<<ss.str()<<endl;
        if (!currentFrame.writeMesh(vertices,ss.str(),0))
            throw std::runtime_error("Failed to write mesh!\nCheck file path!");
    }
    return 0;   
}


int main(){
    
    int res;
    res = execute();
    return res;
}

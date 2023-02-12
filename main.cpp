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
#include "Mesh.h"
#include "cuda/ray_casting_cuda.cuh"

#define USE_ICP 1
// 1 if use icp to optimize pose

/*
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
*/
int execute()
{
    // path to the data
    std::string filenameIn = "../Data/rgbd_dataset_freiburg1_xyz/";

    // path to the output
    std::string filenameBaseOut = "../results/mesh_";
    std::cout << "Initialize virtual sensor..." << std::endl;

    VirtualSensor sensor;
    if (!sensor.Init(filenameIn))
        throw std::runtime_error("Failed to initialize the sensor! Check file path!");
    /* init consts*/
    Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
    Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
    unsigned int width = sensor.GetDepthImageWidth();
    unsigned int height = sensor.GetDepthImageHeight();
    float edgeThreshold = 10;
    bool filtered = true;
    const int max_level = 0; // layer of pyramid
    /*
        configuration of pose estimation
    */
    Eigen::Matrix4f cur_pose = Matrix4f::Identity();
    const float distance_threshold = 0.02f;
    const float angle_threshold = 20.0f;
    std::vector<int> num_iterations = std::vector<int>{10, 5, 4}; // from last to front

    /* init volume*/
    const float grid_len = 0.06;
    VoxelArray volume(std::array<unsigned, 3>{100, 100, 100}, grid_len, Vector3f{-1, -1, -1}, cur_pose);
    /* init return vertices */
    std::vector<Vertex> Cast_Vertices;

    while (sensor.ProcessNextFrame() && sensor.GetCurrentFrameCnt() < 10)
    {
        Matrix4f trajectory = sensor.GetTrajectory(); // not used
        BYTE* colorMapNew = &sensor.GetColorRGBX()[0];
        float* depthMapNew = &sensor.GetDepth()[0];
        Frame currentFrame(depthMapNew, colorMapNew, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered, max_level);

        Pose pose;
        // if (Cast_Vertices.size() != 0)
        // {
        //     // Frame previousFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered, max_level);
        //     auto level_intrinsics = currentFrame.getLevelCameraIntrinstics();
        //     auto level_current_vertices = currentFrame.getPyramidVertex(USE_ICP);
        //     // auto level_previous_vertices = previousFrame.getPyramidVertex(USE_ICP);
        //     auto level_width = currentFrame.getLevelWidth();
        //     auto level_height = currentFrame.getLevelHeight();
        //     // previousFrame = currentFrame;
        //     pose.pose_estimation(level_current_vertices,
        //                          level_previous_vertices,
        //                          level_intrinsics,
        //                          distance_threshold,
        //                          angle_threshold,
        //                          num_iterations,
        //                          level_width,
        //                          level_height,
        //                          max_level,
        //                          cur_pose);
        //     std::cout << "New pose:" << std::endl;
        //     std::cout << cur_pose << std::endl;
        // }

        // surface reconstruction
        std::cout << "Starts Surface Reconstruction" << std::endl;
        float truncationDistance = 0.5;
        clock_t begin = clock();
        volume.SetPose(cur_pose);
        Fusion::SurfaceReconstruction(currentFrame, volume, truncationDistance);
        clock_t end = clock();
        double duration = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "SurfaceReconstruction finished in " << duration << " secs" << std::endl;

        // ray casting
        begin = clock();
        const float step_size = grid_len / 2;
        auto cast_vertices = kinectfusion::ray_casting(volume, step_size, &depthIntrinsics, &cur_pose, width, height);
        end = clock();
        duration = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "ray casting cuda finish in " << duration << " secs" << std::endl;

        // SimpleMesh mesh;
        // for (unsigned row = 0; row < height; ++row)
        //     for (unsigned col = 0; col < width; ++col) {
        //         if (cast_vertices.at(row * width + col).normal.x() != 0)
        //         Mesh::add_point(mesh, util::Vec4to3(cast_vertices.at(row * width + col).position));
        //     std::cout << "Pixel: " << col << ", " << row << " Vertex: " << cast_vertices.at(row * width + col).position.transpose() << " and Normal: " << cast_vertices.at(row * width + col).normal.transpose() << std::endl;
        // }
        // if (!mesh.WriteColoredMesh("../results/" + std::to_string(sensor.GetCurrentFrameCnt()) + "_cast__vertices.off"))
        //     throw std::runtime_error("Out mesh: invalid filename");
        
        // Mesh::export_mesh(volume, "../results/out_mesh_new.off", true);

        // RayCasting cast{width, height, cur_pose, volume};
        // begin = clock();
        // auto [depth, rgba] = cast.SurfacePrediction();
        // end = clock();
        // duration = double(end - begin) / CLOCKS_PER_SEC;
        // std::cout << "ray casting finish in " << duration << " secs" << std::endl;
        /*
        Mesh::export_mesh(volume, "../results/out_mesh.off", true);
        vector<Vertex> vertices = currentFrame.getVertices(USE_ICP); 
        std::cout << "vertices gt!\n";
        if (!currentFrame.writeMesh(vertices, "../results/" + std::to_string(sensor.GetCurrentFrameCnt()) + "out_vertices.off", 0)){
                cout << "Failed to write mesh!\nCheck file path!" << endl;
                return -1;
        }
        */
        // RayCasting cast{width, height, cur_pose, volume};
        // begin = clock();
        // auto [depth, rgba] = cast.SurfacePrediction();
        // if (depthMap != nullptr)
        //     delete[] depthMap;
        // if (colorMap != nullptr)
        //     delete[] colorMap;
        // depthMap = depth;
        // colorMap = rgba;
        // end = clock();
        // duration = double(end - begin) / CLOCKS_PER_SEC;
        // std::cout << "ray casting finish in " << duration << " secs" << std::endl;
        // util::generate_img(sensor, width, height, colorMap, depthMap);
        break;
    }
    // Mesh::export_mesh(volume, "../results/out_mesh.off", true);
    
    return 0;
}

int main()
{

    int res;
    res = execute();
    return res;
}
#pragma once

#include "Volume.hpp"
#include "Frame.h"
#include <memory>

class Fusion {
public:
    /*
        * @description : Loop over every world point and calculate a truncated signed distance value (TSDF), along with a weight
        * @param cur_Frame: current frame
        * @param volume: The container which save the data
        * @param cur_pose: The camera pose of shape £¨4£¬ 4)
        * @param truncationDistance:
        * 
    */
    bool SurfaceReconstruction(const std::shared_ptr<Frame>& cur_Frame, 
                               const std::shared_ptr<Volume>& volume, 
                               const Eigen::Matrix4f& _pose,
                               double truncationDistance);

    private:
        Eigen::Vector3d grid2world(int& x, int& y, int& z, float voxelSize);
        Eigen::Vector2i project2Camera(Eigen::Vector3d& cam_position, Eigen::Matrix3f& Intrinsics);
        float cal_Lamda(Eigen::Vector2i& uv, Eigen::Matrix3f& Intrinsics);
        float cal_SDF(const float&lambda, Eigen::Vector3d& camera_pos, float depth);


};



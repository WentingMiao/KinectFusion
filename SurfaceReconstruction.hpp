#pragma once

#include "Volume.hpp"
#include "Frame.h"
#include <memory>

class Fusion {
public:
    /*
        * @description : Loop over every world point and calculate a truncated signed distance value (TSDF), along with a weight
        *
    */
    bool SurfaceReconstruction(const std::shared_ptr<Frame>& currentFrame, 
                               const std::shared_ptr<Volume>& volume, 
                               const Matrix3f& Intrinsics,
                               const unsigned int& width,
                               const unsigned int& height,
                               const Eigen::Matrix4f& cur_pose,
                               double truncationDistance);

    private:
        Eigen::Vector3d Fusion::grid2camera(int& x, int& y, int& z, int &volumeSize);
        Eigen::Vector2i project2Camera();
        double cal_Lamda(Eigen::Vector2i& uv, Eigen::Matrix3f& Intrinsics);
        double cal_SDF(double&lambda, Eigen::Vector3d& camera_pos, double depth);


};



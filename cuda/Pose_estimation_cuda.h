#pragma once

#include "Pose_estimation.h"
#include "Frame.h"
#include <cuda_runtime.h>
#include <iostream>

    struct Match {
        int cur_idx;
        int prv_idx;
    };

namespace kinectfusion{


std::tuple<MatrixXf, VectorXf> data_association_cuda(     const std::vector<Vertex>& frame_data,
                                const Matrix3f& Intrinsics,
                                const unsigned int& width,
                                const unsigned int& height,
                                std::unordered_map<int, int>& matches,
                                const Eigen::MatrixXf& previous_pose,
                                const Eigen::MatrixXf& current_pose,
                                const std::vector<Vertex>& model_data,
                                const float& distance_threshold,
                                const float& angle_threshold,
                                double& loss
                                );



}

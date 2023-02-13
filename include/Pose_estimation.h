#pragma once

#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include "Frame.h"
#include <Eigen/Dense>
#include <math.h>
// #include <execution>
#include <mutex>
#include <iterator>
#include <vector>
#include <stdexcept>
#include <cuda_runtime.h>
#include <vector_types.h>
#include "Pose_estimation_cuda.h"
#include <tuple>
constexpr bool DEBUG{true};

// struct Match {
//     int cur_idx;
//     int prv_idx;
// };

class Pose{
public:

Pose() = default;

/**
 * @description: main entrance of pose estimation process
 * @param frame_data: vertex, normal and color map of current frame k 
 * @param model_data: vertex, normal and color map of previous frame k-1
 * @param distance_threshold: outlier check
 * @param angle_threshold: outlier check //with angle °
 * @param num_iteration
 * @param width: image size
 * @param height: image size
 * @param pyramid_level
 * @param cur_pose: pose which we need updated
 * @return true if pose_estimation successed
 */

bool pose_estimation(const std::vector<vector<Vertex>>& frame_data,
                     const std::vector<vector<Vertex>>& model_data,
                     const std::vector<Matrix3f>& intrinstics,
                     const float distance_threshold,
                     const float angle_threshold, 
                     const std::vector<int>& num_iteration,
                     const std::vector<int>& width,
                     const std::vector<int>& height,
                     const int& pyramid_level,
                     Eigen::Matrix4f& cur_pose
);

/**
 * @description: Point to Plane ICP step. we update member m_current_pose with incremental
 * @return {*}
 */
void incremental_caculation   ( MatrixXf& A,
                                VectorXf& b);


/**
 * @description: Util. apply transformation to a vertex
 * @return {*}
 */
Vector3f TransformToVertex(Vector3f vertex, Eigen::Matrix4f Transformation);

/**
 * @description: Util. apply transformation to a normal
 * @return {*}
 */
Vector3f TransformToNormal(Vector3f normal, Eigen::Matrix4f Transformation);

private:

Eigen::Matrix4f m_current_pose;

Eigen::Matrix4f m_previous_pose;

};


#endif //POSE_ESTIMATION_H





















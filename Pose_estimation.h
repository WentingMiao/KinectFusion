#pragma once

#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include "Frame.h"
#include <Eigen/Dense>
#include <math.h>
#include <execution>
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
 * @description: data association step. Matched pairs are stored in matches
 * @return {*}
 */
void data_association(const std::vector<Vertex>& frame_data,
                      const Matrix3f& Intrinsics,
                      const unsigned int& width,
                      const unsigned int& height,
                      std::unordered_map<int, int>& matches);

// void data_association_cuda(const std::vector<Vertex>& frame_data,
//                            const Matrix3f &Intrinsics,
//                            const unsigned int& width,
//                            const unsigned int& height,
//                            std::unordered_map<int, int>& matches);
/**
 * @description: outlier check step. Filter out pairs that meet the requirements
 * @return {*}
 */
void outlier_check( const std::vector<Vertex>& frame_data,
                    const std::vector<Vertex>& model_data,
                    std::unordered_map<int, int>& matches,
                    std::unordered_map<int, int>& selected_matches,
                    const float& distance_threshold,
                    const float& angle_threshold,
                    double& error);

/**
 * @description: Point to Plane ICP step. we update member m_current_pose with incremental
 * @return {*}
 */
void incremental_caculation   ( MatrixXf& A,
                                VectorXf& b);


/**
 * @description: Util. Vector 4f to 3f
 * @return {*}
 */
Vector3f Vector4fToVector3f(Vector4f vertex);

/**
 * @description: Util. Vector 3f to 4f
 * @return {*}
 */
Vector4f Vector3fToVector4f(Vector3f vertex);

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





















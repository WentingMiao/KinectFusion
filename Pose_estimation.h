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

constexpr bool DEBUG{true};

class Pose{
public:

Pose() = default;


/**
 * @description: main entrace of pose estimation stage
 * @param {Matrix4f&} pose: input -> pose of last frame, output -> pose of current frame
 * @param {Vertex&} frame_data: current frame data. call Frame::getVertex() to obtain
 * @param {Vertex&} model_data: Vertex Map + Normal Map + Color Map from predict part
 * @param {float} distance_threshold: outlier about depth for data association 
 * @param {float} angle_threshold: outlier about angle for data association 
 * @param {int&} iteration: number of ICP iteration
 * @return {*} true if ICP is successful
 * 
 * @note need change after pyramid added
 */
bool pose_estimation(const std::vector<vector<Vertex>>& frame_data,
                     const std::vector<vector<Vertex>>& model_data,
                     const std::vector<Matrix3f>& intrinstics,
                     const float distance_threshold,
                     const float angle_threshold, //with angle °
                     const std::vector<int>& num_iteration,
                     const std::vector<int>& width,
                     const std::vector<int>& height,
                     const int& pyramid_level,
                     Eigen::Matrix4f& cur_pose
);

void data_association(const std::vector<Vertex>& frame_data,
                      const Matrix3f &Intrinsics,
                      const unsigned int& width,
                      const unsigned int& height,
                      std::unordered_map<int, int>& matches);

void outlier_check( const std::vector<Vertex>& frame_data,
                    const std::vector<Vertex>& model_data,
                    std::unordered_map<int, int>& matches,
                    std::unordered_map<int, int>& selected_matches,
                    const float& distance_threshold,
                    const float& angle_threshold);

void incremental_caculation(const std::vector<Vertex>& frame_data,
                            const std::vector<Vertex>& model_data,
                            std::unordered_map<int, int>& selected_matches,
                            int& iteration_index);


Vector3f Vector4fToVector3f(Vector4f vertex);

Vector4f Vector3fToVector4f(Vector3f vertex);

Vector3f TransformToVertex(Vector3f vertex, Eigen::Matrix4f Transformation);

Vector3f TransformToNormal(Vector3f normal, Eigen::Matrix4f Transformation);

private:


/* 
pose of current frame.
*/

Eigen::Matrix4f m_current_pose;

Eigen::Matrix4f m_previous_pose;

// std::vector<Vertex> m_frame_data;

// std::vector<Vertex> m_model_data;

// unsigned int m_width;

// unsigned int m_height;

// unsigned int m_pyramid_level;

// Matrix3f m_intrinsics;

// float m_distance_threshold;

// float m_angle_threshold;


};


#endif //POSE_ESTIMATION_H





















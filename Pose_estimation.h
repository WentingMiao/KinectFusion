#pragma once

#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include "Frame.h"
#include <math.h>

struct Match {
	int idx;
	float weight;
};


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
bool pose_estimation(const std::vector<Vertex>& frame_data,
                     const std::vector<Vertex>& model_data,
                     const Matrix3f &Intrinsics,
                     const float distance_threshold,
                     const float angle_threshold, //with angle °
                     const int& num_iteration,
                     const unsigned int& width,
                     const unsigned int& height,
                     const unsigned int& pyramid_level,
                     const Eigen::Matrix4f& cur_pose = Matrix4f::Identity()
);

VectorXf estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, 
                                  const std::vector<Vector3f>& targetPoints, 
                                  const std::vector<Vector3f>& targetNormals,
                                  double &error);

private:

/* 
pose of current frame.
only after execute pose_estimation() then get "true" current pose
*/
Eigen::Matrix4f m_current_pose;

/* 
vector fo all poses
which trajectory can generated later
    */
std::vector<Eigen::Matrix4f> m_poses; 

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





















#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H


#include "Frame.h"
#include "Eigen.h"
#include <Eigen/Dense>

struct Match {
	int idx;
	float weight;
};


class Pose{
public:

//TODO constructor with some constant: iterations on pyramid, threshold
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
                     const float angle_threshold,
                     const int& num_iteration,
                     const unsigned int &width,
                     const unsigned int &height,
                     const Eigen::Matrix4f& pose = Matrix4f::Identity()
);

VectorXd estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, 
                                  const std::vector<Vector3f>& targetPoints, 
                                  const std::vector<Vector3f>& targetNormals,
                                  double &error);

private:

/* 
pose of current frame.
only after execute pose_estimation() then get "true" current pose
*/
Eigen::Matrix4f current_pose;

/* 
vector fo all poses
which trajectory can generated later
    */
std::vector<Eigen::Matrix4f> poses; 


};


#endif //POSE_ESTIMATION_H





















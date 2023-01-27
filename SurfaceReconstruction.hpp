#pragma once

#include "Frame.h"
#include <memory>
#include "Voxels.h"

class Fusion {
public:
    /*
        * @description : Loop over every world point and calculate a truncated signed distance value (TSDF), along with a weight
        * @param cur_Frame: current frame
        * @param volume: The container which save the data
        * @param cur_pose: The camera pose of shape(4x4)
        * @param truncationDistance:
        * 
    */
		bool SurfaceReconstruction(Frame& cur_Frame, 
								  VoxelArray& volume,
								   const Eigen::Matrix4f& _pose,
                                   float truncationDistance);

    private:
        Eigen::Vector3f grid2world(int& x, int& y, int& z, int voxelSize);
		//Eigen::Vector3d grid2world(int& x, int& y, int& z, float voxelSize);
        Eigen::Vector2i project2Camera(Eigen::Vector3f& cam_position, Eigen::Matrix3f& Intrinsics);
        float cal_Lamda(Eigen::Vector2i& uv, Eigen::Matrix3f& Intrinsics);
        float cal_SDF(const float&lambda, Eigen::Vector3f& camera_pos, float depth);


};



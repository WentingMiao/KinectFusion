//Surface Reconstruction

//INPUT: depth img, color img, intrinsics, pose
//OUTPUT:Fused TSDF and color volume

#include <iostream>
#include "VirtualSensor.h"
#include "Eigen.h"                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
#include "Frame.h"
#include "Voxels.h"
#include "SurfaceReconstruction.hpp"
#include "vector"

bool Fusion::SurfaceReconstruction(
    const std::shared_ptr<Frame>& cur_Frame,
	const std::shared_ptr<VoxelArray>& volume,
    const Eigen::Matrix4f& _pose,
    double truncationDistance) 
{
    //step1: Initialize
    auto pose = _pose;  //current frame pose 
    auto width = cur_Frame->getWidth(); //volume width
    auto height = cur_Frame->getHeight(); //volume height
    Matrix3f Intrinsics = cur_Frame->getDepthIntrinsics(); //camera intrinsics matrix of shape (3, 3)
    //Matrix4f depthExtrinsics = cur_Frame->getDepthExtrinsics(); //// depth extrinsives

	double truncationDistance = 0.012f;
	float voxelScale = volume->getGridlen(); //voxel size
	std::array<unsigned, 3> volumeSize = volume->getVolumeSize(); //需要一个传递size的函数 返回volume.size

    //step2: traversal all the voxel in the colume
    for (int z = 0; z < volumeSize[2]; z++) { //size(x, y , z)
        for (int y = 0; y < volumeSize[1]; y++) {
            for (int x = 0; x < volumeSize[0]; x++) {

                //step2.1:voxel position in the world coordinates
                Eigen::Vector3d position = grid2world(x, y, z, voxelScale);
				
				//3 dim -> 4 dim
				Vector4f location = volume->xyz2location(position[0], position[1], position[2]); //私有函数

                //step2.2:world coordinates -> camera coordinates

                //Eigen::Matrix3d rotation = pose.block(0, 0, 3, 3);
                //Eigen::Vector3d translation = pose.block(0, 3, 3, 1);
                //Eigen::Vector3d cam_position = rotation * position + translation;    //current camera position
				Eigen::Vector3d cam_position = volume->World2Camera(position);    //current camera position
                // Check1:if the camera could see
                if (cam_position.z() <= 0)
                    continue;

                //step2.3: project to camera coordinates (2d) to get the corresponding pixel coordinate value (x, y)
                Vector2i uv = project2Camera(cam_position, Intrinsics);

                //Check2:if the projection point is correctly projected within the image extent
                if (uv.x() < 0 || uv.x() >= width || uv.y() < 0 || uv.y() >= height)
                    continue;

                //The depth value of the pixel corresponding to the current voxel
                const float depth = cur_Frame->getDepthMap()[uv.x() + (uv.y() * width)];
                // Check3: if depth <=0 
                if (depth <= 0) continue;

                //step2.4:calculate TSDF
                //tsfd = dx(depth) - dv(distance from current voxel to the camera)
         
                const float lambda = cal_Lamda(uv, Intrinsics);
                const float sdf = cal_SDF(lambda, cam_position, depth);
                //const float sdf = (-1.f) * ((1.f / lambda) * cam_position.norm() - depth);

                //SDF Conversion to TSDF
                if (sdf >= -truncationDistance) {

                    //get current TSDF
                    const float new_tsdf = fmin(1.f, sdf / truncationDistance);

                    //get TSDF and weight already stored in the current model
                    //size_t voxel_index = x + (y * volumeSize[0]) + (z * volumeSize[0] * volumeSize[1]);
					const double old_tsdf = volume->GetSDFVal(location);
                    const double old_weight = volume->GetWeightVal(location);

                    //get updated TSDF and weight
                    const double new_weight = 1.0;
                    const double updated_tsdf = (old_weight * old_tsdf + new_weight * new_tsdf) /
                        (old_weight + new_weight);
                    const double updated_weight = old_weight + new_weight;


                    volume->SetSDFVal(location, updated_tsdf);
                    volume->SetWeightVal(location, updated_weight);

					if (sdf <= truncationDistance / 2 && sdf >= -truncationDistance / 2) {

						Vector4uc& voxel_color = volume->GetColorVal(location);
						const Vector4uc image_color = cur_Frame->getColorMap()[uv.x() + (uv.y() * width)];

						voxel_color[0] = (old_weight * voxel_color[0] + new_weight * image_color[0]) /
							(old_weight + new_weight);
						voxel_color[1] = (old_weight * voxel_color[1] + new_weight * image_color[1]) /
							(old_weight + new_weight);
						voxel_color[2] = (old_weight * voxel_color[2] + new_weight * image_color[2]) /
							(old_weight + new_weight);
						voxel_color[3] = (old_weight * voxel_color[3] + new_weight * image_color[3]) /
							(old_weight + new_weight);
						volume->SetColorVal(location, voxel_color);
					}
                    }
                }
            }
        }
    
    return true;

}


Eigen::Vector3d Fusion::grid2world(int &x, int &y, int &z, float voxelScale)
{
    const Eigen::Vector3d position(
		//0.5为体素中心
        (static_cast<double>(x) + 0.5) * voxelScale,
        (static_cast<double>(y) + 0.5) * voxelScale,
        (static_cast<double>(z) + 0.5) * voxelScale);
    return position;
}


Eigen::Vector2i Fusion::project2Camera(Eigen::Vector3d &cam_position, Eigen::Matrix3f& Intrinsics)
{
    Eigen::Vector2i project_pix(
        cam_position.x() / cam_position.z() * Intrinsics(0, 0) + Intrinsics(0, 2),
        cam_position.y() / cam_position.z() * Intrinsics(1, 1) + Intrinsics(1, 2));

    return project_pix;
}

float Fusion::cal_Lamda(Eigen::Vector2i &uv, Eigen::Matrix3f &Intrinsics)
{
    //内参
    float fovX = Intrinsics(0, 0);
    float fovY = Intrinsics(1, 1);
    float cX = Intrinsics(0, 2);
    float cY = Intrinsics(1, 2);
    Eigen::Vector3d xylambda(
        (uv.x() - cX) / fovX,             
        (uv.y() - cY) / fovY,            
        1.f);

    const float lambda = xylambda.norm();
    return lambda;
}


float Fusion::cal_SDF(const float&lambda, Eigen::Vector3d &camera_pos, float depth)
{
    return (-1.f)* ((1.f / lambda) * camera_pos.norm() - depth);
}



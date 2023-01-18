//Surface Reconstruction

//INPUT
//OUTPUT:Fused TSDF and color volume
//Loop over every world point and calculate a truncated signed distance value (TSDF), along with a weight

#include <iostream>
#include "VirtualSensor.h"
#include "Eigen.h"
#include "Frame.h"
#include "SurfaceReconstruction.hpp"

//using Vec2ida = Eigen::Matrix<int, 2, 1, Eigen::DontAlign>;

bool Fusion::SurfaceReconstruction(
    const std::shared_ptr<Frame>& currentFrame,
    const std::shared_ptr<Volume>& volume,
    const Matrix3f& Intrinsics,
    const unsigned int& width,
    const unsigned int& height,
    const Eigen::Matrix4f& cur_pose,
    double truncationDistance) 
{

    auto pose = cur_pose;
    auto width = currentFrame->getWidth();
    auto height = currentFrame->getHeight();
    auto voxelData = volume->getVoxelData();
    auto volumeSize = volume->getVolumeSize();
    int idx = 0;

    Eigen::Matrix3d rotation = pose.block(0, 0, 3, 3);  //  旋转矩阵
    Eigen::Vector3d translation = pose.block(0, 3, 3, 1);  //平移向量

    //step2: traversal all the voxel in the colume
    for (int z = 0; z < volumeSize.z(); z++) {
        for (int y = 0; y < volumeSize.y(); y++) {
            for (int x = 0; x < volumeSize.x(); x++) {



                //step2.1:grid coordinates -> camera coordinates -- voxel position in the world coordinates
                Eigen::Vector3d position = grid2camera(x, y, z, volumeSize);

                //step2.2:camera coordinates -> world coordinates -- voxel position in current coordinate
                Eigen::Vector3d camera_pos = rotation * position + translation;
                // Check1:if the camera could see
                if (camera_pos.z() <= 0)
                    continue;

                Matrix4f depthExtrinsics = currentFrame->getDepthExtrinsics();

                // calculate project point  in depthmap
                Vector2i uv = project2Camera(camera_pos);

                /// Check2:if the projection point is correctly projected within the image extent
                if (uv.x() < 0 || uv.x() >= width || uv.y() < 0 || uv.y() >= height)
                    continue;

                //const float depth = depth_image.ptr(uv.y())[uv.x()];
                    //if (depth <= 0)
                //continue;

                const double depth = currentFrame->getDepthMap()[uv.x() + (uv.y() * width)];
                // Check3: if depth <=0 
                if (depth <= 0) continue;

                //Calculate lambda
                const float lambda = cal_Lamda(uv, depthExtrinsics);

                //Calculate SDF 
                const float sdf = cal_SDF();
                const float sdf = (-1.f) * ((1.f / lambda) * camera_pos.norm() - depth);

                //Calculate TSDF(SDF Conversion to TSDF)
                if (sdf >= -truncationDistance) {

                    //get current TSDF
                    const float new_tsdf = fmin(1.f, sdf / truncationDistance);

                    //get TSDF and weight already stored in the current model
                    size_t voxel_index = x + (y * volumeSize.x()) + (z * volumeSize.x() * volumeSize.y());
                    const double old_tsdf = volume->getVoxelData()[voxel_index].tsdf;
                    const double old_weight = volume->getVoxelData()[voxel_index].weight;

                    //get updated TSDF and weight
                    const double new_weight = 1.0;
                    const double updated_tsdf = (old_weight * old_tsdf + new_weight * new_tsdf) /
                        (old_weight + new_weight);
                    const double updated_weight = old_weight + new_weight;


                    volume->getVoxelData()[voxel_index].tsdf = updated_tsdf;
                    volume->getVoxelData()[voxel_index].weight = updated_weight;


                    }
                }
            }
        }
    
    return true;

}





Eigen::Vector3d Fusion::grid2camera(int &x, int &y, int &z, int &volumesize)
{
    const Eigen::Vector3d position(
        (static_cast<double>(x) + 0.5) * volumeSize,
        (static_cast<double>(y) + 0.5) * volumeSize,
        (static_cast<double>(z) + 0.5) * volumeSize);
    return position;
}


Eigen::Vector2i Fusion::project2Camera(Eigen::Vector3d camera_position)
{
    //
}

double Fusion::cal_Lamda(Eigen::Vector2i &uv, Eigen::Matrix3f &Intrinsics)
{
    double fovX = Intrinsics(0, 0);
    double fovY = Intrinsics(1, 1);
    double cX = Intrinsics(0, 2);
    double cY = Intrinsics(1, 2);
    Eigen::Vector3d xylambda(
        (uv.x() - cX) / fovX,             
        (uv.y() - cY) / fovY,            
        1.f);

    const double lambda = xylambda.norm();
    return lambda;
}

double Fusion::cal_SDF(double&lambda, Eigen::Vector3d &camera_pos, double depth)
{
    return (-1.f)* ((1.f / lambda) * camera_pos.norm() - depth);
}


















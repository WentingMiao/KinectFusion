// Surface Reconstruction

// INPUT: depth img, color img, intrinsics, pose
// OUTPUT:Fused TSDF and color volume

#include <iostream>
#include <fstream>
#include "VirtualSensor.h"
#include "Eigen.h"
#include "Frame.h"
#include "Voxels.h"s
#include "SurfaceReconstruction.hpp"
#include "vector"

namespace
{
    Vector3f Vec4to3(const Vector4f vec)
    {
        return Vector3f{vec(0) / vec(3), vec(1) / vec(3), vec(2) / vec(3)};
    }

    Vector4f Vec3to4(const Vector3f vec)
    {
        return Vector4f{vec(0), vec(1), vec(2), 1.f};
    }

    Eigen::Vector2i project2Camera(Eigen::Vector3f cam_position, Eigen::Matrix3f Intrinsics)
    {
        Eigen::Vector2i project_pix(
            cam_position.x() / cam_position.z() * Intrinsics(0, 0) + Intrinsics(0, 2),
            cam_position.y() / cam_position.z() * Intrinsics(1, 1) + Intrinsics(1, 2));
        return project_pix;
    }

    float cal_Lamda(Eigen::Vector2i uv, Eigen::Matrix3f &Intrinsics)
    {
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

    float cal_SDF(const float lambda, Eigen::Vector3f camera_pos, float depth)
    {
        return (-1.f) * ((1.f / lambda) * camera_pos.norm() - depth);
    }

    bool IfInit(const Vector3f initlocation, Vector3f origin)
    {
        if(initlocation(0) != (-0.25)){
            return false;
        }
        if(initlocation(1) != (-0.25)){
            return false;
        }
        if(initlocation(2) != (0.75)){
            return false;
        }
        return true;
    }

}



bool Fusion::SurfaceReconstruction(
    Frame &cur_Frame,
    VoxelArray &volume,
    float truncationDistance)
{
    // step1: Initialize
    auto width = cur_Frame.getWidth();                    // volume width
    auto height = cur_Frame.getHeight();                  // volume height
    Matrix3f Intrinsics = cur_Frame.getDepthIntrinsics(); // camera intrinsics matrix of shape (3, 3)
    float voxelScale = volume.getGridlen();
    auto volumeSize = volume.GetSize();
    auto origin = volume.GetOrigin();

    // std::cout<<"origin(0)"<<origin(0)<<endl;
    // std::cout<<"origin(1)"<<origin(1)<<endl;
    // std::cout<<"origin(2)"<<origin(2)<<endl;
    
    float maxweight = 10;

    ofstream myout("sdf.txt");
    if (myout.is_open()) {
        myout << "is opened,out from myout" << endl;
	}

    myout<<"origin(0)"<<origin(0)<<endl;
    myout<<"origin(1)"<<origin(1)<<endl;
    myout<<"origin(2)"<<origin(2)<<endl;

    myout << "width" << width <<std::endl; 
    myout << "height" << height <<std::endl; 
    myout << "voxelScale" << voxelScale <<std::endl; 
    myout << "volumeSize" << volumeSize[0] <<std::endl; 
    myout << "origin" << origin <<std::endl; 
    int i=1;

    // step2: traversal all the voxel in the colume
    for (int z = 0; z < volumeSize[2]; z++)
    {
        for (int y = 0; y < volumeSize[1]; y++)
        {
            for (int x = 0; x < volumeSize[0]; x++)
            {
                // step2.1:voxel position in the world coordinates
                Vector4f location = volume.xyz2location(x , y, z);

                // step2.2:world coordinates -> camera coordinates
                Vector4f cam_position = volume.World2Camera(location); // current camera position
                // Check1:if the camera could see
                if (cam_position.z() <= 0)
                    continue;

                // step2.3: project to camera coordinates (2d) to get the corresponding pixel coordinate value (x, y)
                Vector2i uv = project2Camera(cam_position.block<3, 1>(0, 0), Intrinsics);

                myout << "x:" << x << " y:" << y << " z:" << z << endl;


                //std::cout << "uv " << uv << std::endl; 
                myout << "uv.x:" << uv.x() << " uv.y:" << uv.y() << endl; 
                // Check2:if the projection point is correctly projected within the image extent
                if (uv.x() < 0 || uv.x() >= width || uv.y() < 0 || uv.y() >= height)
                    continue;

                // The depth value of the pixel corresponding to the current voxel
                const float depth = cur_Frame.getDepthMap()[uv.x() + (uv.y() * width)];
                //std::cout << "depth " << depth << std::endl; 
                //std::cout << "1" << std::endl; 

                
                // Check3: if depth <=0
                if (depth <= 0)
                    continue;
                myout << "if depth>0,depth:" << depth << endl; 
                // step2.4:calculate TSDF
                // tsfd = dx(depth) - dv(distance from current voxel to the camera)
                // cout << "start writing at location: " << uv.transpose() << "with grid " << Vector3i{x, y, z}.transpose() << endl;
               
                //std::cout << "depth " << depth << std::endl; 
                const float lambda = cal_Lamda(uv, Intrinsics);
                const float sdf = cal_SDF(lambda, Vec4to3(cam_position), depth);
                

                //myout<<"locationX:"<<Vec4to3(location)(0)<<"locationY:"<<Vec4to3(location)(1)<<"locationZ:"<<Vec4to3(location)(2)<<endl;

                if(i == 1){
                   // myout<<"init"<<endl;
                    //init tsdf&weight
                    volume.InitWeightVal(location);
                    volume.InitSDFVal(location);
                    i++;
                }
                myout << "SDF:" << sdf << endl; 
                // SDF Conversion to TSDF
                if (sdf >= -truncationDistance)
                {
                    // get current TSDF
                    const float new_tsdf = fmin(1.f, sdf / truncationDistance);
                    //const float new_weight = 1.0;
                    //cout << "new_weight" << new_weight << endl;

                    // get TSDF and weight already stored in the current model
                    // size_t voxel_index = x + (y * volumeSize[0]) + (z * volumeSize[0] * volumeSize[1]);
                    //const float old_tsdf = volume.GetSDFVal(location);
                    const float old_weight = volume.GetWeightVal(location);
                    const float new_weight =  1.0f;

                   
                    // get updated TSDF and weight
                    const float updated_tsdf = (old_weight * updated_tsdf + new_weight * new_tsdf) /
                                               (old_weight + new_weight);
                    const float updated_weight = old_weight + new_weight;
                    

                    volume.SetSDFVal(location, updated_tsdf);
                    volume.SetWeightVal(location, updated_weight);

                    //test
                    myout << "if sdf >= -truncationDistance" << endl;
                    myout << "new_tsdf" << new_tsdf << endl;
                    //myout << "old_tsdf" << old_tsdf << endl;
                    myout << "old_weight" << old_weight << endl;
                    myout << "new_weight" << new_weight << endl;
                    //myout << updated_tsdf << endl;
                    myout << "updated_weight:" << updated_weight << endl;

                    //cout << "updated_tsdf:" << updated_tsdf << endl;
                    //volume.showtsdf(location);


                    if (sdf <= truncationDistance / 2 && sdf >= -truncationDistance / 2)
                    {

                        Vector4uc voxel_color = volume.GetColorVal(location);
                        const Vector4uc image_color = cur_Frame.getColorMap()[uv.x() + (uv.y() * width)];

                        voxel_color[0] = (old_weight * voxel_color[0] + new_weight * image_color[0]) /
                                         (old_weight + new_weight);
                        voxel_color[1] = (old_weight * voxel_color[1] + new_weight * image_color[1]) /
                                         (old_weight + new_weight);
                        voxel_color[2] = (old_weight * voxel_color[2] + new_weight * image_color[2]) /
                                         (old_weight + new_weight);
                        voxel_color[3] = (old_weight * voxel_color[3] + new_weight * image_color[3]) /
                                         (old_weight + new_weight);
                        volume.SetColorVal(location, voxel_color);
                    }

                    myout<<endl;
                }
            }
        }
    }
    
    myout.close();
    return true;
}

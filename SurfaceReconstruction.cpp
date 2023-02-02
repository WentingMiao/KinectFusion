// Surface Reconstruction

// INPUT: depth img, color img, intrinsics, pose
// OUTPUT:Fused TSDF and color volume

#include <iostream>
#include <fstream>
#include "VirtualSensor.h"
#include "Eigen.h"
#include "Frame.h"
#include "Voxels.h"
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
        return depth - ((1.f / lambda) * camera_pos.norm());
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
	
    ofstream camout("cam.txt");
    ofstream my2out("tsdfvalue.txt");
    ofstream depthout("depth.txt");
    ofstream locationout("location.txt");
    ofstream sdfout("sdf.txt");
    ofstream tsdfout("tsdf.txt");


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
		locationout << "idx" << volume.location2idx(location) << endl;
                locationout << "location" << location << endl;


                // step2.2:world coordinates -> camera coordinates
                Vector4f cam_position = volume.World2Camera(location); // current camera position
		camout << "idx" << volume.location2idx(location) << endl;
                camout << "cam_position:" << cam_position << endl;

                // Check1:if the camera could see
                if (cam_position.z() <= 0)
                    continue;

                // step2.3: project to camera coordinates (2d) to get the corresponding pixel coordinate value (x, y)
                Vector2i uv = project2Camera(cam_position.block<3, 1>(0, 0), Intrinsics);

		    
                // Check2:if the projection point is correctly projected within the image extent
                if (uv.x() < 0 || uv.x() >= width || uv.y() < 0 || uv.y() >= height)
                    continue;

                // The depth value of the pixel corresponding to the current voxel
                const float depth = cur_Frame.getDepthMap()[uv.x() + (uv.y() * width)];
		depthout << "idx" << volume.location2idx(location) << endl;
                depthout << "depth " << depth << endl; 

                
                // Check3: if depth <=0
                if (depth <= 0)
                    continue;
		    
                // step2.4:calculate TSDF
                // tsfd = dx(depth) - dv(distance from current voxel to the camera)
                // cout << "start writing at location: " << uv.transpose() << "with grid " << Vector3i{x, y, z}.transpose() << endl;

                const float lambda = cal_Lamda(uv, Intrinsics);
                //const float sdf = cal_SDF(lambda, Vec4to3(cam_position), depth);
		const float sdf = depth - cam_position.z() ;
		sdfout << "idx" << volume.location2idx(location) << endl;
                sdfout << "sdf " << sdf << endl; 
                

                // SDF Conversion to TSDF
                if (sdf >= -truncationDistance)
                {
		
		    if(i == 1){
                        //init tsdf&weight
                        volume.InitWeightVal(location);
                        volume.InitSDFVal(location);
                        i++;
                    }
                    // get current TSDF
                    const float new_tsdf = fmin(1.f, sdf / truncationDistance);
		    //const float new_tsdf2 = fmax(-1, fmin(1.f, sdf / truncationDistance));


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
                    tsdfout << "idx" << volume.location2idx(location) << endl;
                    tsdfout << "updated_tsdf" << updated_tsdf << endl;


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
                }
		my2out<<"idx"<<volume.xyz2idx(x, y, z)<<endl;
                my2out<<"tsdf"<<volume.GetSDFVal(location)<<endl;

            }
        }
    }
    
    //my3out.close();
    depthout.close();
    sdfout.close();
    tsdfout.close();
    locationout.close();
    camout.close();
    return true;
}

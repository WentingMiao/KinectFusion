#pragma once
#include "Frame.h"
#include "FreeImageHelper.h"
#include <array>
#include <tuple>
#include "Voxels.h"
#include "util.h"
/*

Ray casting module:
    Given Camera Pose and TSDF, generate a vertices with color and depth
    Role:
        TSDF is in world coordinate
        Volume integration integrate points of world coord into tsdf
        Ray casting should predict an image from of "predicted camera" from last pose

    use:
    RayCasting cast{size_t width, size_t height, const Eigen::Matrix4f &Pose, VoxelArray &tsdf_arr};
    auto imgs = cast.SurfacePrediction();

*/

class RayCasting
{
public:
    inline RayCasting(size_t width, size_t height, const Eigen::Matrix4f &Pose, VoxelArray &tsdf_arr) : _width(width), _height(height), _Pose(Pose), tsdf{tsdf_arr}
    {
    }
    std::tuple<float *, BYTE *> SurfacePrediction(); // get surface location
private:
    Vector4f Pixel2World(unsigned int x, unsigned int y); // transform pixel location to world location
    Vertex CastPixel(const unsigned x, const unsigned y); // obtain the vertex corresponding to pixel
    float World2Depth(Vector4f location);                 // obtain depth info
    // linear interpolation to obtain color and location of vertex
    Vertex interpolation(const Vector4f &loc1, const Vector4f &loc2);
    const size_t _width;
    const size_t _height;
    Matrix4f _Pose;
    VoxelArray &tsdf;
};
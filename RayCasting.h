#pragma once
#include "Frame.h"
#include "FreeImageHelper.h"
#include <array>
#include <tuple>
#include <memory>
#include "Voxels.h"
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

    TODO: support search distance limit!!!

    meeting
    0. 代码在干嘛？
    1. lambda在干嘛？
    2. 如何可视化？
    3. 重构代码？
*/

// unsigned char manipulation functions
Vector4uc uc_addition(const Vector4uc &a, const Vector4uc &b);
Vector4uc uc_subtraction(const Vector4uc &a, const Vector4uc &b);
Vector4uc uc_elementwise_mult(float a, const Vector4uc &b);

class RayCasting
{
public:
    inline RayCasting(size_t width, size_t height, const Eigen::Matrix4f &Pose, VoxelArray &tsdf_arr) : _width(width), _height(height), _Pose(Pose), tsdf{tsdf_arr} 
    {}
    std::tuple<std::unique_ptr<float>, std::unique_ptr<BYTE>> SurfacePrediction(); // get surface location
private:
    Vector4f Pixel2World(unsigned int x, unsigned int y);                           // transform pixel location to world location
    Vertex CastPixel(const unsigned x, const unsigned y);                           // obtain the vertex corresponding to pixel
    struct Ray
    {
        inline Ray(const Vector4f &origin, const Vector4f &direction, float step_size, float begin_distance = 0)
            : _origin{origin.block<3, 1>(0, 0)}, _step_size{step_size}, _distance{begin_distance}
        {
            Vector3f tmp{direction(0) / direction(3), direction(1) / direction(3), direction(2) / direction(3)};
            _direction = tmp.normalized();
        };
        Vector4f getLocation(); // get current location
        inline void step()      // update location by one step
        {
            _distance += _step_size;
        }
        float _step_size;
        float _distance;
        Vector3f _origin;
        Vector3f _direction;
    };
    // linear interpolation to obtain color and location of vertex
    Vertex interpolation(const Ray& r, const Vector4f &loc1, const Vector4f &loc2); 
    const size_t _width;
    const size_t _height;
    Matrix4f _Pose;
    VoxelArray &tsdf;
};
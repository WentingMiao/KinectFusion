#pragma once
#include "Frame.h"
#include "Eigen.h"
#include "vector"
#define DEBUG
/*
TODO:
    2. world与camera坐标转换如果常用可以独立成camera类
*/
struct VoxelElement
{
    float sdf = MINF;
    float weight = 0;
    Vector4uc color = Vector4uc(0, 0, 0, 0);
};
// consistent with "Frame.h",
// invalid SDF value: MINF,
// invalid weight value: 0,
// invalid Color value: Vector4uc(0, 0, 0, 0)

class VoxelInterface
{
public:
    VoxelInterface(float grid_len, Matrix4f Pose) : _grid_len{grid_len}, _Pose{Pose} {};
    virtual ~VoxelInterface() = default;
    virtual void SetWeightVal(const Vector4f& location, float weight) = 0;
    virtual void SetSDFVal(const Vector4f& location, float sdf) = 0;
    virtual void SetColorVal(const Vector4f& location, Vector4uc color) = 0;
    // change voxel value / insert new voxel according to world location
    virtual float GetWeightVal(const Vector4f& location) const = 0;
    virtual float GetSDFVal(const Vector4f& location) const = 0;
    virtual Vector4uc GetColorVal(const Vector4f& location) const = 0;
    virtual bool isValidLocation(const Vector4f& world_location) const = 0;
    Vector4f World2Camera(const Vector4f& world_location);
    Vector4f Camera2World(const Vector4f& camera_location);
    float getGridlen() const ;
#ifndef DEBUG
protected:
#endif
    const float _grid_len;
    Matrix4f _Pose;        // camera pose
};

class VoxelArray : public VoxelInterface
{
public:
    VoxelArray() = delete;
    VoxelArray(std::array<unsigned, 3> size, float grid_len, Vector3f origin, Matrix4f Pose) : 
        VoxelInterface{grid_len, Pose}, _origin{origin}, _size{size}, voxel{size[0] * size[1] * size[2]},
        _valid_location_range {{_origin(0), _origin(0) + _grid_len * size[0]}, 
        {_origin(1), _origin(1) + _grid_len * size[1]}, {_origin(2), _origin(2) + _grid_len * size[2]}} {};
    virtual ~VoxelArray() = default;
    virtual bool isValidLocation(const Vector4f& location) const override;

    /* setting value in voxel */
    virtual void SetWeightVal(const Vector4f& location, float weight) override;
    virtual void SetSDFVal(const Vector4f& location, float sdf) override;
    virtual void SetColorVal(const Vector4f& location, Vector4uc color) override;

    /* getting value in voxel*/
    virtual float GetWeightVal(const Vector4f& location) const override;
    virtual float GetSDFVal(const Vector4f& location) const override;
    virtual Vector4uc GetColorVal(const Vector4f& location) const override;

#ifndef DEBUG
private:
#endif
    Vector3f _origin; // corner of grid , {-3, -3, 0} may be appropriate
    unsigned location2idx(const Vector4f& location) const; // world location to voxel index
    Vector4f idx2location(const unsigned idx) const;
    std::array<unsigned, 3> _size;
    std::vector<VoxelElement> voxel;
    Matrix<float, 3, 2> _valid_location_range; // valid range of world location
};
#include "Voxels.h"

Vector4f VoxelInterface::World2Camera(Vector4f world_location)
{
    return _Pose * world_location;
}

Vector4f VoxelInterface::Camera2World(Vector4f camera_location)
{
    return _Pose.inverse() * camera_location;
}

float VoxelInterface::getGridlen()
{
    return _grid_len;
}

bool VoxelArray::isValidLocation(Vector4f location)
{
    if ((location(0) / location(3)) < _valid_location_range(0, 0) || (location(0) / location(3)) > _valid_location_range(0, 1))
        return false;
    if ((location(1) / location(3)) < _valid_location_range(1, 0) || (location(1) / location(3)) > _valid_location_range(1, 1))
        return false;
    if ((location(2) / location(3)) < _valid_location_range(2, 0) || (location(2) / location(3)) > _valid_location_range(2, 1))
        return false;
    return true;
}

unsigned VoxelArray::location2idx(Vector4f location)
{
    if (!isValidLocation(location))
        throw std::out_of_range("Invalid world location");
    unsigned x = std::floor((location(0) / location(3) - _origin(0)) / _grid_len);
    unsigned y = std::floor((location(1) / location(3) - _origin(1)) / _grid_len);
    unsigned z = std::floor((location(2) / location(3) - _origin(2)) / _grid_len);
    return x + y * _size[0] + z * _size[0] * _size[1];
}

void VoxelArray::SetWeightVal(Vector4f location, float weight)
{
    unsigned idx = location2idx(location);
    voxel[idx].weight = weight;
}

void VoxelArray::SetSDFVal(Vector4f location, float sdf)
{
    unsigned idx = location2idx(location);
    voxel[idx].sdf = sdf;
}

void VoxelArray::SetColorVal(Vector4f location, Vector4uc color)
{
    unsigned idx = location2idx(location);
    voxel[idx].color = color;
}

float VoxelArray::GetWeightVal(Vector4f location)
{
    unsigned idx = location2idx(location);
    return voxel[idx].weight;
}

float VoxelArray::GetSDFVal(Vector4f location)
{
    unsigned idx = location2idx(location);
    return voxel[idx].sdf;
}
Vector4uc VoxelArray::GetColorVal(Vector4f location)
{
    unsigned idx = location2idx(location);
    return voxel[idx].color;
}
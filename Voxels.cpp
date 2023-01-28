#include "Voxels.h"

Vector4f VoxelInterface::World2Camera(const Vector4f& world_location)
{
    return _Pose * world_location;
}

Vector4f VoxelInterface::Camera2World(const Vector4f& camera_location)
{
    return _Pose.inverse() * camera_location;
}

float VoxelInterface::getGridlen() const
{
    return _grid_len;
}

bool VoxelArray::isValidLocation(const Vector4f& location) const
{
    if ((location(0) / location(3)) < _valid_location_range(0, 0) || (location(0) / location(3)) > _valid_location_range(0, 1))
        return false;
    if ((location(1) / location(3)) < _valid_location_range(1, 0) || (location(1) / location(3)) > _valid_location_range(1, 1))
        return false;
    if ((location(2) / location(3)) < _valid_location_range(2, 0) || (location(2) / location(3)) > _valid_location_range(2, 1))
        return false;
    return true;
}

unsigned VoxelArray::location2idx(const Vector4f& location) const
{
    if (!isValidLocation(location))
        throw std::out_of_range("Invalid world location");
    unsigned x = std::floor((location(0) / location(3) - _origin(0)) / _grid_len);
    unsigned y = std::floor((location(1) / location(3) - _origin(1)) / _grid_len);
    unsigned z = std::floor((location(2) / location(3) - _origin(2)) / _grid_len);
    return x + y * _size[0] + z * _size[0] * _size[1];
}

Vector4f VoxelArray::idx2location(const unsigned idx) const {
    /*
    -2.656 -> -2.70000005
    -2.65 belong to [-2.7, -2.65]
    but -2.7 have precision issue, which become -2.700005 that fall into another voxel!
    This is partly why we should set location of voxel to its center, not corner
    */
    if (idx >= voxel.size())
        throw std::out_of_range("Invalid index: " + std::to_string(idx));
    std::array<unsigned, 3> xyz = idx2xyz(idx);
    return Vector4f{xyz[0] * _grid_len + _origin(0) + _grid_len / 2, xyz[1] * _grid_len + _origin(1) + _grid_len / 2, 
    xyz[2] * _grid_len + _origin(2) + _grid_len / 2, 1};
}

Vector4f VoxelArray::xyz2location(const unsigned x, const unsigned y, const unsigned z) const {
    if (x >= _size[0] || y >= _size[1] || z >= _size[2])
        throw std::out_of_range("Invalid index");
    return Vector4f{x * _grid_len + _origin(0) + _grid_len / 2, y * _grid_len + _origin(1) + _grid_len / 2, 
    z * _grid_len + _origin(2) + _grid_len / 2, 1};
}

unsigned VoxelArray::xyz2idx(const unsigned x, const unsigned y, const unsigned z) const {
    return x + y * _size[0] + z * _size[0] * _size[1];
}

std::array<unsigned, 3> VoxelArray::idx2xyz(const unsigned idx) const {
    unsigned x = idx % _size[0];
    unsigned y = std::floor((idx % (_size[0] * _size[1]) - x) / _size[0]);
    unsigned z = std::floor(idx / (_size[0] * _size[1]));
    return std::array<unsigned, 3>{x, y, z};
}

void VoxelArray::SetWeightVal(const Vector4f& location, float weight)
{
    unsigned idx = location2idx(location);
    voxel[idx].weight = weight;
}

void VoxelArray::SetSDFVal(const Vector4f& location, float sdf)
{
    unsigned idx = location2idx(location);
    voxel[idx].sdf = sdf;
}

void VoxelArray::SetColorVal(const Vector4f& location, Vector4uc color)
{
    unsigned idx = location2idx(location);
    voxel[idx].color = color;
}

float VoxelArray::GetWeightVal(const Vector4f& location) const
{
    unsigned idx = location2idx(location);
    return voxel[idx].weight;
}

float VoxelArray::GetSDFVal(const Vector4f& location) const
{
    unsigned idx = location2idx(location);
    return voxel[idx].sdf;
}

Vector4uc VoxelArray::GetColorVal(const Vector4f& location) const
{
    unsigned idx = location2idx(location);
    return voxel[idx].color;
}
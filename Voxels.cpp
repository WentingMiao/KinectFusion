#include "Voxels.h"

Vector4f VoxelInterface::World2Camera(Vector4f world_location) {
    return _Pose * world_location;
}

Vector4f VoxelInterface::Camera2World(Vector4f camera_location) {
    return _Pose.inverse() * camera_location;
}

unsigned VoxelArray::location2idx(Vector4f location) {
    float x = std::floor(location(0) / location(3) / _grid_len);
    float y = std::floor(location(1) / location(3) / _grid_len);
    float z = std::floor(location(2) / location(3) / _grid_len);
    return x * _size[0] + y * _size[1] + z;
}
void VoxelArray::SetWeightVal(Vector4f location, float weight) {
    unsigned idx = location2idx(location);
    voxel[idx].weight = weight;
}
void VoxelArray::SetSDFVal(Vector4f location, float sdf) {
    unsigned idx = location2idx(location);
    voxel[idx].sdf = sdf;
}
void VoxelArray::SetColorVal(Vector4f location, Vector4uc color) {
    unsigned idx = location2idx(location);
    voxel[idx].color = color;
}

float VoxelArray::GetWeightVal(Vector4f location) {
    unsigned idx = location2idx(location);
    return voxel[idx].weight;
}

void VoxelArray::GetSDFVal(Vector4f location) {
    unsigned idx = location2idx(location);
    return voxel[idx].sdf;
}
void VoxelArray::GetColorVal(Vector4f location) {
    unsigned idx = location2idx(location);
    return voxel[idx].color;
}
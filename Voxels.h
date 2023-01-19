#pragma once
#include "Frame.h"
#include "Eigen.h"
#include "vector"
/*
TODO:
    1. worldcoord与voxel对应只实现了线性缩放，只能处理一段为0的空间
        （eg: 2-100 -> 2/factor - 100/factor）, index没有被好好利用, 负坐标甚至会出现非法idx
        在interface加入“原点”, 以实现仿射变换,解决上述问题
    2. world与camera坐标转换如果常用可以独立成camera类
*/
struct VoxelElement {
    float sdf = MINF;
    float weight = 0;
    Vector4uc color = Vector4uc(0, 0, 0, 0);
};
// consistent with "Frame.h", 
// invalid SDF value: MINF, 
// invalid weight value: 0,
// invalid Color value: Vector4uc(0, 0, 0, 0)

class VoxelInterface {
    public:
        VoxelInterface(float grid_len, Matrix4f Pose): _grid_len{grid_len}, _Pose{Pose} {};
        virtual void SetWeightVal(Vector4f location, float weight) = 0;
        virtual void SetSDFVal(Vector4f location, float sdf) = 0;
        virtual void SetColorVal(Vector4f location, Vector4uc color) = 0;
        // change voxel value / insert new voxel according to world location
        virtual float GetWeightVal(Vector4f location) = 0;
        virtual float GetSDFVal(Vector4f location) = 0;
        virtual Vector4uc GetColorVal(Vector4f location) = 0;
    protected: 
        Vector4f World2Camera(Vector4f world_location);
        Vector4f Camera2World(Vector4f camera_location);
        const float _grid_len; // do we need to support rectangle voxel?
        Matrix4f _Pose; // camera pose
};

class VoxelArray: public VoxelInterface {
    public:
        VoxelArray() = delete;
        VoxelArray::VoxelArray(std::array<unsigned, 3> size, float grid_len, Matrix4f Pose):
            VoxelInterface{grid_len, Pose}, _size{size}, voxel{size[0] * size[1] * size[2]} {};
        virtual void SetWeightVal(Vector4f location, float weight) override;
        virtual void SetSDFVal(Vector4f location, float sdf) override;
        virtual void SetColorVal(Vector4f location, Vector4uc color) override;
        virtual float GetWeightVal(Vector4f location) override;
        virtual float GetSDFVal(Vector4f location) override;
        virtual Vector4uc GetColorVal(Vector4f location) override;
    private:
        unsigned location2idx(Vector4f location);
        std::array<unsigned, 3> _size;
        std::vector<VoxelElement> voxel;
};
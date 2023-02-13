#pragma once
#include "Frame.h"
#include "Eigen.h"
#include "vector"
#include "util.h"

// consistent with "Frame.h",icp_state
// invalid SDF value: MINF,
// invalid weight value: 0,
// invalid Color value: Vector4uc(0, 0, 0, 0)
struct VoxelElement
{
    float sdf = 1; // agree with tuencate distance
    float weight = 0;
    Vector4uc color = Vector4uc(0, 0, 0, 0);
};

class VoxelArray
{
public:
    VoxelArray() = delete;
    VoxelArray(std::array<unsigned, 3> size, float grid_len, Vector3f origin, Matrix4f Pose) : _grid_len{grid_len}, _Pose{Pose}, _origin{origin}, _size{size}, voxel{size[0] * size[1] * size[2]}
    {
        _valid_location_range = Matrix<float, 3, 2>{{_origin(0), _origin(0) + _grid_len * size[0]},
                                                    {_origin(1), _origin(1) + _grid_len * size[1]},
                                                    {_origin(2), _origin(2) + _grid_len * size[2]}};
        // std::cout << "Voxel info\n";
        // std::cout << "grid len: " << grid_len << std::endl;
        // std::cout << "location range: \n" << _valid_location_range << std::endl;
    };
    ~VoxelArray() = default;
    bool isValidLocation(const Vector4f &location) const;

    /* set value in voxel */
    void SetWeight(const Vector4f &location, float weight);
    void SetSDF(const Vector4f &location, float sdf);
    void SetColor(const Vector4f &location, Vector4uc color);

    /* get value in voxel */
    float GetWeight(const Vector4f &location) const;
    float GetSDF(const Vector4f &location) const;
    Vector4uc GetColor(const Vector4f &location) const;

    /* get meta info */
    Vector3f GetOrigin() const { return _origin; }
    auto GetSize() const { return _size; }
    auto GetDimX() const { return _size[0]; }
    auto GetDimY() const { return _size[1]; }
    auto GetDimZ() const { return _size[2]; }

    /* camera transformation */
    Vector4f World2Camera(const Vector4f &world_location);
    Vector4f Camera2World(const Vector4f &camera_location);
    void SetPose(const Matrix4f pose) { _Pose = pose; }

    /* location manipulation*/
    float getGridlen() const;
    // world location to voxel index
    unsigned location2idx(const Vector4f &location) const;
    // voxel index to world location
    Vector4f idx2location(const unsigned idx) const;
    // xyz to world location
    Vector4f xyz2location(const unsigned x, const unsigned y, const unsigned z) const;
    // xyz to index
    unsigned xyz2idx(const unsigned x, const unsigned y, const unsigned z) const;
    // index to xyz
    std::array<unsigned, 3> idx2xyz(const unsigned idx) const;

    std::vector<VoxelElement> voxel;
private:
    Vector3f _origin;                          // corner of grid , {-3, -3, 0} may be appropriate
    std::array<unsigned, 3> _size;             // number of voxel on xyz dimension
    const float _grid_len;
    Matrix<float, 3, 2> _valid_location_range; // valid range of world location
    Matrix4f _Pose;
};
#include <gtest/gtest.h>
#include "../Voxels.h"
#include <iostream>
#include <random>

class voxel_test : public ::testing::Test
{
public:
    voxel_test() : tsdf{std::array<unsigned, 3>{600, 600, 600}, 0.05, Vector3f{-3, -3, 0}, Matrix4f{}} {}
    ~voxel_test() override = default;
    VoxelArray tsdf;
    void SetUp() override
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
};

Vector4f Rand_valid_location(Matrix<float, 3, 2> range)
{
    std::random_device seed;
    std::ranlux48 engine(seed());
    std::uniform_real_distribution<> Xdistrib(range(0, 0), range(0, 1));
    std::uniform_real_distribution<> Ydistrib(range(1, 0), range(1, 1));
    std::uniform_real_distribution<> Zdistrib(range(2, 0), range(2, 1));
    return Vector4f{Xdistrib(engine), Ydistrib(engine), Zdistrib(engine), 1};
}

bool compare_float(float a, float b)
{
    if (1e-3 > std::abs(a - b))
        return true;
    return false;
}

// TEST_F(voxel_test, Init)
// {
//     auto arr = std::array<unsigned, 3>{600, 600, 600};
//     // check value
//     ASSERT_EQ(tsdf._origin, Vector3f(-3, -3, 0));
//     ASSERT_EQ(tsdf._size, arr);
//     ASSERT_TRUE(compare_float(0.05, tsdf._grid_len));
//     ASSERT_EQ(Vector3f(-3, -3, 0), tsdf._origin);
// }



// TEST_F(voxel_test, Setvalue)
// {
//     // set and then get values in random locations
//     for (int i = 0; i < 40000; ++i)
//     {
//         auto tmp_loc = Rand_valid_location(tsdf._valid_location_range);
//         auto tmp_val = 100 * (tmp_loc(0) - tmp_loc(1));
//         tsdf.SetWeightVal(tmp_loc, tmp_val);
//         tsdf.SetSDFVal(tmp_loc, tmp_val);
//         ASSERT_TRUE(compare_float(tsdf.GetWeightVal(tmp_loc), tmp_val));
//         ASSERT_TRUE(compare_float(tsdf.GetSDFVal(tmp_loc), tmp_val));
//     }
// }

TEST_F(voxel_test, LocationIdxTransformation)
{
    // set and then get values in random locations
    for (int i = 0; i < 40000; ++i)
    {
        auto tmp_loc = Rand_valid_location(tsdf._valid_location_range);
        unsigned idx = tsdf.location2idx(tmp_loc);
        ASSERT_EQ(idx, tsdf.location2idx(tsdf.idx2location(idx)));
    }
}
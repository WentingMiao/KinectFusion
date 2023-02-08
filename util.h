#pragma once
#include "Eigen.h"
#include "VirtualSensor.h"

namespace util
{
    inline Vector3f Vec4to3(const Vector4f vec)
    {
        return Vector3f{vec(0) / vec(3), vec(1) / vec(3), vec(2) / vec(3)};
    }

    inline Vector4f Vec3to4(const Vector3f vec)
    {
        return Vector4f{vec(0), vec(1), vec(2), 1.f};
    }

    inline void generate_img(VirtualSensor& sensor, unsigned int width, unsigned int height, BYTE* colorMap, float* depthMap) {
        FreeImageB rgbdimg{width, height};
        for (int i = 0; i < width * height * 4; i++)
            rgbdimg.data[i] = colorMap[i];
        rgbdimg.SaveImageToFile("../results/rgbd" + std::to_string(sensor.GetCurrentFrameCnt()) + ".png");
        FreeImage depthimg{width, height, 1};
        for (int i = 0; i < width * height; i++)
            depthimg.data[i] = depthMap[i];
        auto depth_intensity = depthimg.ConvertToIntensity();
        depth_intensity.SaveImageToFile("../results/depth" + std::to_string(sensor.GetCurrentFrameCnt()) + ".png");
    }

    class Camera
    {
    public:
        Camera(Matrix3f intrinsics, Matrix4f extrainsics): Intrinsics{intrinsics}, Extrinsics{extrainsics}, 
            InvExtrinsics{extrainsics.inverse()} {}

        // Vector3i camera2pixel(const Vector4f location) const {

        // }
        // Vector3i pixel2camera(const Vector3i pixel) const {

        // }
        
        Vector4f world2camera(const Vector4f location) const
        {
            return Extrinsics * location;
        }
        Vector4f camera2world(const Vector4f location) const
        {
            return InvExtrinsics * location;
        }

        Matrix3f& getIntrinsics()
        {
            return Intrinsics;
        }

    private:
        Matrix3f Intrinsics;
        Matrix4f Extrinsics;
        Matrix4f InvExtrinsics;
    };
}

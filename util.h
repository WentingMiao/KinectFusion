#pragma once
#include "Eigen.h"
namespace util {
    inline Vector3f Vec4to3(const Vector4f vec)
    {
        return Vector3f{vec(0) / vec(3), vec(1) / vec(3), vec(2) / vec(3)};
    }

    inline Vector4f Vec3to4(const Vector3f vec)
    {
        return Vector4f{vec(0), vec(1), vec(2), 1.f};
    }

    class Camera {
    public:
        // Vector3i camera2pixel(const Vector4f location) const {

        // }
        // Vector3i pixel2camera(const Vector3i pixel) const {
            
        // }
        Vector4f world2camera(const Vector4f location) const {
            return Extrinsics * location; 
        }
        Vector4f camera2world(const Vector4f location) const {
            return InvExtrinsics * location; 
        }

        Matrix3f getIntrinsics() {
            return Intrinsics;
        }
    private:
        Matrix3f Intrinsics;
        Matrix4f Extrinsics;
        Matrix4f InvIntrinsics;
        Matrix4f InvExtrinsics;
    };
}
    
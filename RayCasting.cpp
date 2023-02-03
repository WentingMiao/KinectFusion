#include "RayCasting.h"
#include <iostream>
#include <vector>
namespace
{
    Vector4uc uc_subtraction(const Vector4uc &a, const Vector4uc &b)
    {
        Vector4uc tmp{a(0) - b(0), a(1) - b(1), a(2) - b(2), a(3) - b(3)};
        return tmp;
    }

    Vector4uc uc_addition(const Vector4uc &a, const Vector4uc &b)
    {
        Vector4uc tmp{static_cast<u_char>(a(0) + b(0)), static_cast<u_char>(a(1) + b(1)), static_cast<u_char>(a(2) + b(2)), static_cast<u_char>(a(3) + b(3))};
        return tmp;
    }

    Vector4uc uc_elementwise_mult(float a, const Vector4uc &b)
    {
        Vector4uc tmp{static_cast<u_char>(a * b(0)), static_cast<u_char>(a * b(1)), static_cast<u_char>(a * b(2)), static_cast<u_char>(a * b(3))};
        return tmp;
    }
}

std::tuple<std::unique_ptr<float>, std::unique_ptr<BYTE>> RayCasting::SurfacePrediction()
{
    unsigned width = _width;
    unsigned height = _height;
    std::unique_ptr<float> depth{new float[width * height]};
    std::unique_ptr<BYTE> rgba{new BYTE[width * height * 4]};
    for (unsigned row = 0; row < height; ++row)
        for (unsigned col = 0; col < width; ++col)
        {
            Vertex ret = CastPixel(row, col);
            rgba.get()[4 * (width * row + col)] = ret.color(0);
            rgba.get()[4 * (width * row + col) + 1] = ret.color(1);
            rgba.get()[4 * (width * row + col) + 2] = ret.color(2);
            rgba.get()[4 * (width * row + col) + 3] = ret.color(3);
            if (ret.depth == MINF) // remove MINF in depth image
                depth.get()[width * row + col] = 0;
            else
                depth.get()[width * row + col] = ret.depth * 5000;
        }
    return std::make_tuple(std::move(depth), std::move(rgba));
}

Vertex RayCasting::CastPixel(const unsigned x, const unsigned y)
{
    Ray r{tsdf.Camera2World(Vector4f{0, 0, 0, 0}), Pixel2World(x, y), tsdf.getGridlen() / 2, 0.0f};
    Vector4f lastLocation;
    Vector4f currLocation;
    while (tsdf.isValidLocation(r.getLocation()))
    {
        lastLocation = currLocation;
        currLocation = r.getLocation();
        if (tsdf.GetSDFVal(lastLocation) * tsdf.GetSDFVal(currLocation) < 0) // surface
            return interpolation(r, lastLocation, currLocation);
        else
            r.step();
    }
    return Vertex{};
}

Vertex RayCasting::interpolation(const Ray &r, const Vector4f &loc1, const Vector4f &loc2)
{
    /*
    untested
    loc1: last
    loc2: curr
    Linear interpolation to approximate surface location
        -f(x2) / f(x1) - f(x2) = x2x* / x2x1
        factor := -f(x2) / f(x1) - f(x2)
        x* = x2 + factor * x2x1
    */
    std::cout << "Interpolation between " << loc1.transpose() << " and " << loc2.transpose() << " with sdf " << tsdf.GetSDFVal(loc1) << " and " << tsdf.GetSDFVal(loc2)
              << std::endl;
    float factor = (tsdf.GetSDFVal(loc2) / (tsdf.GetSDFVal(loc2) - tsdf.GetSDFVal(loc1)));
    float depth = r._distance - factor * r._step_size;
    Vector4f est_location = loc2 + factor * (loc1 - loc2);
    Vector4uc color = uc_addition(
        tsdf.GetColorVal(loc2),
        uc_elementwise_mult(
            factor,
            uc_subtraction(
                tsdf.GetColorVal(loc1),
                tsdf.GetColorVal(loc2))));
    return Vertex{est_location, color, depth};
}

Vector4f RayCasting::Pixel2World(unsigned int x, unsigned int y)
{
    float fX = 525.0f;
    float fY = 525.0f;
    float cX = 319.5f;
    float cY = 239.5f;
    float z = 1; // value of z doesn't matter
    return _Pose * Vector4f{z * (x - cX) / fX, z * (y - cY) / fY, z, 1.f};
}

Vector4f RayCasting::Ray::getLocation()
{
    Vector4f loc;
    loc.block<3, 1>(0, 0) = _origin + _distance * _direction;
    loc(3) = 1;
    return loc;
}
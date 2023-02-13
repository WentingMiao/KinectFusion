#include "RayCasting.h"
#include <iostream>
#include "Mesh.h"
#include <vector>

namespace
{
    Vector4uc uc_subtraction(const Vector4uc &a, const Vector4uc &b)
    {
        Vector4uc tmp{static_cast<u_char>(a(0) - b(0)), static_cast<u_char>(a(1) - b(1)), static_cast<u_char>(a(2) - b(2)), static_cast<u_char>(a(3) - b(3))};
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

std::tuple<float*, BYTE*> RayCasting::SurfacePrediction()
{
    unsigned width = _width;
    unsigned height = _height;
    float* depth = new float[width * height];
    BYTE* rgba = new BYTE[width * height * 4];
    
    #ifdef DEBUG
    SimpleMesh mesh;
    ofstream outfile("../results/cpu_cast.log");
    #endif
    for (unsigned row = 0; row < height; ++row)
        for (unsigned col = 0; col < width; ++col)
        {
            Vertex ret = CastPixel(col, row);
            #ifdef DEBUG
            outfile << "Pixel: " << col << ", " << row << ", location: " <<  ret.position.transpose() << std::endl;
            if (ret.position.x() != MINF)
                Mesh::add_point(mesh, util::Vec4to3(ret.position), Vector4uc{0, 255, 0, 255});
            #endif
            rgba[4 * (width * row + col)] = ret.color(0);
            rgba[4 * (width * row + col) + 1] = ret.color(1);
            rgba[4 * (width * row + col) + 2] = ret.color(2);
            rgba[4 * (width * row + col) + 3] = ret.color(3);
            if (ret.depth == MINF) // remove MINF in depth image
                depth[width * row + col] = 0;
            else
                depth[width * row + col] = ret.depth;
        }
    #ifdef DEBUG
    auto time = clock();
    if (!mesh.WriteColoredMesh("../results/" + std::to_string(static_cast<float>(time)) + "_cpu_cast_vertices.off"))
        throw std::runtime_error("Out mesh: invalid filename");
    #endif
    return std::make_tuple(depth, rgba);
}

Vertex RayCasting::CastPixel(const unsigned x, const unsigned y)
{
    auto start = tsdf.World2Camera({0, 0, 0, 0});
    auto direction = (tsdf.World2Camera(Vector4f{static_cast<float>(x), static_cast<float>(y), 1, 1}) - tsdf.World2Camera({0, 0, 0, 0})).normalized();
    auto step_size = tsdf.getGridlen() / 2;
    Vector4f lastLocation;
    Vector4f currLocation = start;

    while (tsdf.isValidLocation(currLocation) && tsdf.isValidLocation(currLocation + direction * step_size))
    {
        lastLocation = currLocation;
        currLocation += direction * step_size;
        if (tsdf.GetSDF(lastLocation) * tsdf.GetSDF(currLocation) < 0)
            return interpolation(lastLocation, currLocation);
        else
            currLocation += direction;
    }
    return Vertex{};
}

Vertex RayCasting::interpolation(const Vector4f &loc1, const Vector4f &loc2)
{
    /*
    loc1: last
    loc2: curr
    Linear interpolation to approximate surface location
        -f(x2) / f(x1) - f(x2) = x2x* / x2x1
        factor := -f(x2) / f(x1) - f(x2)
        x* = x2 + factor * x2x1
    */
    float factor = (tsdf.GetSDF(loc2) / (tsdf.GetSDF(loc2) - tsdf.GetSDF(loc1)));
    Vector4f est_location = loc2 + factor * (loc1 - loc2);
    Vector4uc color = uc_addition(
        tsdf.GetColor(loc2),
        uc_elementwise_mult(
            factor,
            uc_subtraction(
                tsdf.GetColor(loc1),
                tsdf.GetColor(loc2))));
    return Vertex{est_location, color, World2Depth(est_location)};
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

float RayCasting::World2Depth(Vector4f location)
{
    location = tsdf.World2Camera(location); // back to camera
    location /= location(3); // homogeneous back to heterogeneous
    return location(2);
}
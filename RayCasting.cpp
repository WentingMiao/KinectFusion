#include "RayCasting.h"

std::vector<Vertex> RayCasting::SurfacePrediction()
{
    std::vector<Vertex> vertices(_width * _height);
    for (int i = 0; i < _width; ++i)
        for (int j = 0; j < _height; ++j)
        {
            vertices[_width * i + j] = CastPixel(i, j);
        }
    computeNormal(vertices);
    return vertices;
}

void RayCasting::computeNormal(std::vector<Vertex> &vertices)
{
    for (unsigned int row = 1; row < _height - 1; row++)
    {
        for (unsigned int col = 1; col < _width - 1; col++)
        {
            int idx = row * _width + col;

            // 1. search for points in the neighbourhood
            Vector4f point = vertices[idx].position;
            Vector4f leftPoint = vertices[idx - 1].position;
            Vector4f rightPoint = vertices[idx + 1].position;
            Vector4f upperPoint = vertices[idx - _width].position;
            Vector4f lowerPoint = vertices[idx + _width].position;

            // 2. compute principal component
            Vector4f du = vertices[idx + 1].position - vertices[idx - 1].position;
            Vector4f dv = vertices[idx + _width].position - vertices[idx - _width].position;

            // we set normal to invalid when vertex are too far away from its neigbours
            if (du.norm() < 10 || dv.norm() < 10)
            {
                // getting the norm by cross product of two vectors made up of neighbours
                Vector3f normal = cross(du, dv);
                normal = normal.normalized();
                // 3. normalize the norm
                vertices[idx].normal = normal;
            }
        }
    }
}

Vector4f RayCasting::Pixel2World(unsigned int x, unsigned int y)
{
    float fX = 525.0f;
    float fY = 525.0f;
    float cX = 319.5f;
    float cY = 239.5f;
    float z = 0.1; // value of z doesn't matter
    return _Pose * Vector4f{z * (x - cX) / fX, z * (y - cY) / fY, z, 1.f};
}

Vertex RayCasting::CastPixel(const unsigned x, const unsigned y)
{
    /*
    Building Ray:
        transform origin and (x, y, 1) into world coordinate
    Surface prediction:
        step() until meet surface or end
        if meet surface -> location prediction, return vertex
    */
    Ray r{tsdf.Camera2World(Vector4f{0, 0, 0, 0}), Pixel2World(x, y), tsdf.getGridlen() / 2, 0.0f};
    Vector4f lastLocation;
    Vector4f currLocation;
    while (tsdf.isValidLocation(r.getLocation())) // TODO: is valid distance
    {
        lastLocation = currLocation;
        currLocation = r.getLocation();
        if (tsdf.GetSDFVal(lastLocation) * tsdf.GetSDFVal(currLocation) <= 0) // surface
            return interpolation(lastLocation, currLocation);
        else
            r.step();
    }
    return Vertex{};
}
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

Vertex RayCasting::interpolation(const Vector4f &loc1, const Vector4f &loc2)
{
    /*
    Linear interpolation to approximate surface location
        -f(x2) / f(x1) - f(x2) = x2x* / x2x1
        factor := -f(x2) / f(x1) - f(x2)
        x* = x2 + factor * x2x1
    */

    float factor = (tsdf.GetSDFVal(loc2) / (tsdf.GetSDFVal(loc2) - tsdf.GetSDFVal(loc1)));
    Vector4f est_location = loc2 + factor * (loc1 - loc2);
    Vector4uc color = uc_addition(
        tsdf.GetColorVal(loc2),
        uc_elementwise_mult(
            factor,
            uc_subtraction(
                tsdf.GetColorVal(loc1),
                tsdf.GetColorVal(loc2))));
    return Vertex{est_location, color};
}

Vector4f RayCasting::Ray::getLocation()
{
    Vector4f loc;
    loc.block<3, 1>(0, 0) = _origin + _distance * _direction;
    loc(3) = 1;
    return loc;
}
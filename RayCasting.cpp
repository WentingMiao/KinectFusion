#include "RayCasting.h"

std::vector<Vertex> RayCasting::Surface_prediction()
{
    std::vector<Vertex> vertices(_width * _height);
    for (int i = 0; i < _width; ++i)
        for (int j = 0; j < _height; ++j)
        {
            vertices[_width * i + j] = CastPixel(i, j);
        }
    return vertices;
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

Vertex RayCasting::CastPixel(int x, int y)
{
    /*
    Building Ray:
        transform origin and (x, y, 1) into world coordinate
    Surface prediction:
        step() until meet surface or end
        if meet surface -> location prediction, return vertex
    */
    Ray r{tsdf.Camera2World(Vector4f{0, 0, 0, 0}), Pixel2World(x, y), tsdf.getGridlen() / 2};
    Vector4f lastLocation;
    Vector4f currLocation;
    while (tsdf.isValidLocation(r.getLocation())) // todo: is valid distance
    //!
    {
        lastLocation = currLocation;
        currLocation = r.getLocation();
        if (tsdf.GetSDFVal(lastLocation) * tsdf.GetSDFVal(currLocation) <= 0) // surface
        {
            return Interpolation(lastLocation, currLocation);
        }
        else
            r.step();
    }
    return Vertex{};
}
Vector4uc uc_subtraction(const Vector4uc& a, const Vector4uc& b) {
    Vector4uc tmp{a(0) - b(0), a(1) - b(1), a(2) - b(2), a(3) - b(3)};
    return tmp;
}

Vector4uc uc_addition(const Vector4uc& a, const Vector4uc& b) {
    Vector4uc tmp{a(0) + b(0), a(1) + b(1), a(2) + b(2), a(3) + b(3)};
    return tmp;
}

Vector4uc uc_elementwise_mult(float a, const Vector4uc& b) {
    Vector4uc tmp{static_cast<u_char>(a * b(0)), static_cast<u_char>(a * b(1)), static_cast<u_char>(a * b(2)), static_cast<u_char>(a * b(3))};
    return tmp;
}

Vertex RayCasting::Interpolation(const Vector4f &loc1, const Vector4f &loc2)
{
    // -f(x2) / f(x1) - f(x2) = x2x* / x2x1
    // factor := -f(x2) / f(x1) - f(x2)
    // x* = x2 + factor * x2x1
    float factor = (tsdf.GetSDFVal(loc2) / (tsdf.GetSDFVal(loc2) - tsdf.GetSDFVal(loc1)));
    Vector4f est_location = loc2 + factor * (loc1 - loc2);
    Vector4uc color = uc_addition(
        tsdf.GetColorVal(loc2), 
        uc_elementwise_mult(
            factor, 
            uc_subtraction(
                tsdf.GetColorVal(loc1), 
                tsdf.GetColorVal(loc2)
            )
        )
    );

    Vertex ret;
    //{est_location, color, };
    ret.position = est_location;
    return ret;
}

Vector4f RayCasting::Ray::getLocation()
{
    Vector4f loc;
    loc.block<3, 1>(0, 0) = _origin + _distance * _direction;
    loc(3) = 1;
    return loc;
}

void RayCasting::Ray::step()
{
    _distance += _step_size;
}
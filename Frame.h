#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex{
    // position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;

	// color stored as 4 unsigned char
	Vector4uc color;

    //normal of the point
    Vector3f normal;
};

class Frame{
    private:
        unsigned int width;
        unsigned int height;

        float* depthMap;
        BYTE* colorMap;

        Matrix3f depthIntrinsics;
        Matrix4f depthExtrinsics;
        Matrix4f trajectory;

        float edgeThreshold;

       // std::vector<Eigen::Vector4f> points_c;
        std::vector<Eigen::Vector4f> points_w;
        std::vector<Vertex> vertices;
        
        Vector3f cross(Vector4f v1, Vector4f v2);

    public:

        Frame(VirtualSensor sensor, float edgeThreshold);
        

        // get points of camera cooridinate
        std::vector<Eigen::Vector4f> fromPixelToWorld();
  
        std::vector<Vertex>  getVertices();

        float* getDepthMap();
     
};
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

    
    public:

        //Frame(VirtualSensor& sensor, float edgeThreshold);
        Frame(float* depthMap,  BYTE* colorMap, Eigen::Matrix3f &depthIntrinsics, Eigen::Matrix4f &depthExtrinsics, 
         Eigen::Matrix4f &trajectory, unsigned int width,unsigned int height, float edgeThreshold = 2);


        // get points of camera cooridinate
        // std::vector<Eigen::Vector4f> fromPixelToWorld();
  
        std::vector<Vertex> getVertices();

        std::vector<float> getDepthMap();
        std::vector<Vector4uc> getColorMap();

        unsigned int getWidth();
        unsigned int getHeight();

     private:
        unsigned int _width;
        unsigned int _height;
    
        
        std::vector<float> _depthMap;
        // BYTE* _colorMap;
        std::vector<Vector4uc> _colorMap;

        Matrix3f _depthIntrinsics;
        Matrix4f _depthExtrinsics;
        Matrix4f _trajectory;

        float _edgeThreshold;

       // std::vector<Eigen::Vector4f> points_c;
        std::vector<Eigen::Vector4f> points_w;
        std::vector<Vertex> vertices;
        
        // Vector3f cross(Vector4f v1, Vector4f v2);

};
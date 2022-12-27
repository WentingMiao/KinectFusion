#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex{
    // position stored as 4 floats (4th component represents supposed to be 1.0)
    // Vector4f(MINF, MINF, MINF, MINF) represents invalid point
	Vector4f position;

	// color stored as 4 unsigned char,  Vector4uc(0,0,0,0) represents invalid color
	Vector4uc color; 

    //normal of the point , Vector3f(MINF, MINF, MINF) represents invalid normal
    Vector3f normal;
};

class Frame{

    
    public:

        //Frame constructor
        Frame(float* depthMap,  BYTE* colorMap, Eigen::Matrix3f &depthIntrinsics, Eigen::Matrix4f &depthExtrinsics, 
         Eigen::Matrix4f &trajectory, unsigned int width,unsigned int height, float edgeThreshold = 2);


        std::vector<Eigen::Vector4f> getCameraPoints();// get points of camera cooridinate
  
        std::vector<Vertex> getVertices(); // get vertices of the frame
        std::vector<float> getDepthMap(); //get depthmap of the frame
        std::vector<Vector4uc> getColorMap(); //get the colormap of the frame, type is Vector4uc 

        unsigned int getWidth(); //get width of depth image
        unsigned int getHeight(); //get height of depth image

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
        // std::vector<Eigen::Vector4f> points_w;
        std::vector<Vertex> vertices;
        
      

};
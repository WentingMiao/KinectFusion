#include "Eigen.h"
#include "VirtualSensor.h"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

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
         Eigen::Matrix4f &trajectory, unsigned int width,unsigned int height, float edgeThreshold, bool filtered, unsigned int maxLevel);


        void applyBilateralFilter(vector<float>& originalDepth, vector<float>& outputDepth, unsigned int width, unsigned int height);

        void buildDepthPyramid(vector<float>& originalMap, vector<vector<float>>& outputMap,unsigned int maxLevel);
        void buildColorPyramid(vector<Vector4uc>& originalMap, vector<vector<Vector4uc>>& outputMap, unsigned int maxLevel);


        bool writeMesh(vector<Vertex>& vectices, const string& filename, unsigned int level);

        vector<vector<Vertex>>  getPyramidVertex();

        /*setter function*/
        void setFilterSize(int size);
        void setSigmaColor(double sigmaColor);
        void setSigmaSpace(double sigmaSpace);

        /* getter function */
        vector<Eigen::Vector4f> getCameraPoints();// get points of camera cooridinate
        vector<Vertex> getVertices(); // get vertices of the frame
        vector<float> getDepthMap(); //get depthmap of the frame
        vector<Vector4uc> getColorMap(); //get the colormap of the frame, type is Vector4uc 



        Matrix3f getLevelCameraIntrinstics(unsigned int level);
        Matrix4f getDepthExtrinsics(); // get depth extrinsives
        Matrix3f getDepthIntrinsics(); // get camera intrinsives
        unsigned int getWidth(); //get width of depth image
        unsigned int getHeight(); //get height of depth image

     private:
        unsigned int _width;
        unsigned int _height;
        
        // bilateralFilter values
        int _dValue= 5; //Diameter of each pixel neighborhood that is used during filtering. 
        
       // double _sigmaColor = 0.033f; //Filter sigma in the color space. 
        unsigned int _maxLevel;
        double _sigmaColor = 9.0f; 
        double _sigmaSpace = 525.0f; //Filter sigma in the coordinate space

        vector<float> _depthMap;
        
        vector<Vector4uc> _colorMap;

        Matrix3f _depthIntrinsics;
        Matrix4f _depthExtrinsics;
        Matrix4f _trajectory;

        float _edgeThreshold;

        vector<float> _pyramidWidth;
        vector<float> _pyramidHeight;

        vector<vector<Vertex>> _pyramidVertex;
        vector<vector<float>> _pyramidDepthMap;

        vector<Matrix3f> _allDepthIntrinsic;
        vector<Vertex> vertices;
        
};
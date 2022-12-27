#include "Frame.h"


// Frame::Frmae( float* _depthMap,  BYTE* colorMap, Matrix3f _depthIntrinsics, Matrix4f _depthExtrinsicsInv, unsigned int _width, unsigned int _height)
//Frame::Frame(VirtualSensor& sensor, float _edgeThreshold)
    // : _sensor(sensor),_width(sensor.GetDepthImage_width()), _height(sensor.GetDepthImage_height()),             ,_depthMap(sensor.GetDepth()),colorMap(sensor.GetColorRGBX()), _depthIntrinsics(sensor.Get_depthIntrinsics()),_edgeThreshold(_edgeThreshold) ,_depthExtrinsics(sensor.Get_depthExtrinsics()), _trajectory(sensor.Get_trajectory())
//    {
        // std::cout << _depthMap[20] <<std::endl;
        // _depthMap = sensor.GetDepth();
        // std::cout << _width <<std::endl;
       // _sensor = sensor;
//    }


//constructor of Frame
Frame::Frame(float* depthMap,  BYTE* colorMap, Eigen::Matrix3f &depthIntrinsics, Eigen::Matrix4f &depthExtrinsics, 
         Eigen::Matrix4f &trajectory, unsigned int width,unsigned int height, float edgeThreshold )
: _width(width), _height(height), _depthIntrinsics(depthIntrinsics), _depthExtrinsics(depthExtrinsics), _edgeThreshold(edgeThreshold),_trajectory(trajectory)
{
    _depthMap = std::vector<float>(_width * _height);
    _colorMap = std::vector<Vector4uc>(_width * _height);
    for (unsigned int i = 0 ; i < _width * _height; i++){
         _depthMap[i] = _depthMap[i];
         _colorMap[i] = Vector4uc(colorMap[4*i], colorMap[4*i+1], colorMap[4*i+2], colorMap[4*i+3]);
    }

}

// std::vector<Eigen::Vector4f> Frame::fromPixelToWorld(){
    
//     std::vector<Eigen::Vector4f> points( _width*_height );

//     float fX = _depthIntrinsics(0, 0);
// 	float fY = _depthIntrinsics(1, 1);
// 	float cX = _depthIntrinsics(0, 2);
// 	float cY = _depthIntrinsics(1, 2);

//     for(unsigned int row = 0; row < _height; row++){
//         for(unsigned int col = 0; col < _width; col++){
//             int idx = row * _width + col;
//             float z = _depthMap[idx]; 
            
//             if(z == MINF){
//                 points[idx] = Vector4f(MINF, MINF, MINF, MINF);
// 			}else{
//                 float u = col;
// 				float v = row;
// 				float x = z*(u-cX)/fX; 
// 				float y = z*(v-cX)/fY; 
                
// 				// point_c = Vector4f(x, y, x, 1.0);
//                 points[idx] =  _trajectory.inverse() * _depthExtrinsics.inverse() * Vector4f(x, y, x, 1.0);
                
//             }
//         }
//     }
//     return points;

// }

Vector3f cross(Vector4f v1, Vector4f v2){
    Vector3f v1_ = v1.head<3>();
    Vector3f v2_ = v2.head<3>();
    return v1_.cross(v2_);
}

std::vector<Vertex>  Frame::getVertices(){

    std::vector<Vertex> vertices(_width * _height);

    /* get the 3d position and color */
    float fX = _depthIntrinsics(0, 0);
	float fY = _depthIntrinsics(1, 1);
	float cX = _depthIntrinsics(0, 2);
	float cY = _depthIntrinsics(1, 2);

    for(unsigned int row = 0; row < _height; row++){
        for(unsigned int col = 0; col < _width; col++){
            int idx = row * _width + col;
            float z = _depthMap[idx]; 
            Vector4f point_c, point_w;
            Vector4uc color = _colorMap[idx];

            if(z == MINF){
                point_w = Vector4f(MINF, MINF, MINF, MINF);
                color = Vector4uc(0,0,0,0);
			}else{
                float u = col;
				float v = row;

				float x = z*(u-cX)/fX; 
				float y = z*(v-cX)/fY; 
                
				point_c = Vector4f(x, y, z, 1.0);
                point_w =  _trajectory.inverse() * _depthExtrinsics.inverse() * point_c;
                

            }
            vertices[idx].color = color;
            vertices[idx].position = point_w;
        }
    }

    //compute normal

    for(unsigned int row = 1; row < _height - 1; row++){
        for(unsigned int col = 1; col < _width - 1; col++){
            int idx = row * _width + col;

            Vector4f point = vertices[idx].position;
            Vector4f leftPoint = vertices[idx - 1].position;
            Vector4f rightPoint = vertices[idx + 1].position;
            Vector4f upperPoint = vertices[idx - _width].position;
            Vector4f lowerPoint = vertices[idx + _width].position;


            Vector4f du =  vertices[idx + 1].position - vertices[idx - 1].position;
            Vector4f dv =  vertices[idx + _width].position - vertices[idx - _width].position;

            //we set normal to invalid when vertex are too far away from its neigbours
            if(!du.allFinite()||!dv.allFinite()||du.norm() >= _edgeThreshold||dv.norm() >= _edgeThreshold){
                vertices[idx].normal = Vector3f(MINF, MINF, MINF);
            }
            else{
                Vector3f normal = cross(du,dv);
                normal = normal.normalized();
                vertices[idx].normal = normal;
            }

        }
    }


    //We set invalid normals for border regions
    for (int u = 0; u < _width; ++u) {
        vertices[u].normal = Vector3f(MINF, MINF, MINF);
        vertices[u + (_height - 1) * _width].normal = Vector3f(MINF, MINF, MINF);
    }

    for (int v = 0; v < _height; ++v) {
        vertices[v * _width].normal = Vector3f(MINF, MINF, MINF);
        vertices[(_width - 1) + v * _width].normal = Vector3f(MINF, MINF, MINF);
    }


    return vertices;


}
// std::vector<Eigen::Vector4f> fromCameraToWorld(std::vector<Eigen::Vector4f> points_c){
    
// }




std::vector<float> Frame::getDepthMap(){
    return _depthMap;
}

std::vector<Vector4uc> Frame::getColorMap(){
    return _colorMap;
}


unsigned int Frame::getWidth(){
    return _width;
}

unsigned int Frame::getHeight(){
    return _height;
}


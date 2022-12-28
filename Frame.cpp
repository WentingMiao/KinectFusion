#include "Frame.h"





//constructor of Frame
Frame::Frame(float* depthMap,  BYTE* colorMap, Eigen::Matrix3f &depthIntrinsics, Eigen::Matrix4f &depthExtrinsics, 
         Eigen::Matrix4f &trajectory, unsigned int width,unsigned int height, float edgeThreshold )
: _width(width), _height(height), _depthIntrinsics(depthIntrinsics), _depthExtrinsics(depthExtrinsics), _edgeThreshold(edgeThreshold),_trajectory(trajectory)
{
    _depthMap = std::vector<float>(_width * _height);
    _colorMap = std::vector<Vector4uc>(_width * _height);
    for (unsigned int i = 0 ; i < _width * _height; i++){
         _depthMap[i] = depthMap[i];

         // color is stored as RGBX in row major (4 byte values per pixel) 
         // so the size of colorMap is 4 * width * height
         // we convert it to 4 unsigned char type
         _colorMap[i] = Vector4uc(colorMap[4*i], colorMap[4*i+1], colorMap[4*i+2], colorMap[4*i+3]);
    }

}


std::vector<Eigen::Vector4f> Frame::getCameraPoints(){
    
    std::vector<Eigen::Vector4f> points( _width*_height );

    float fX = _depthIntrinsics(0, 0);
	float fY = _depthIntrinsics(1, 1);
	float cX = _depthIntrinsics(0, 2);
	float cY = _depthIntrinsics(1, 2);

    for(int row = 0; row < _height; row++){
        for(int col = 0; col < _width; col++){
            int idx = row * _width + col;
            float z = _depthMap[idx]; 
            
            if(z == MINF){
                points[idx] = Vector4f(MINF, MINF, MINF, MINF);
			}else{
                float u = col;
				float v = row;
				float x = ( u - cX) / fX * z; 
				float y = ( v - cX) / fY * z; 
                
				// point_c = Vector4f(x, y, z, 1.0);
                points[idx] =  _trajectory.inverse() * _depthExtrinsics.inverse() * Vector4f(x, y, z, 1.0);
                
            }
        }
    }
    return points;

}

Vector3f cross(Vector4f v1, Vector4f v2){
    Vector3f v1_ = v1.head<3>();
    Vector3f v2_ = v2.head<3>();
    Vector3f normal = v1_.cross(v2_);
    return normal;
}

std::vector<Vertex>  Frame::getVertices(){

    std::vector<Vertex> vertices(_width * _height);

    /* get the 3d position and color */
    float fX = _depthIntrinsics(0, 0);
	float fY = _depthIntrinsics(1, 1);
	float cX = _depthIntrinsics(0, 2);
	float cY = _depthIntrinsics(1, 2);
    
    // if the depth value at idx is invalid (MINF) we set position and color of the point invalid
	// otherwise we apply back-projection and transform the point to world space
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

                // Back-projection to camera space.
				float x = z*(u-cX)/fX; 
				float y = z*(v-cX)/fY; 
                
                //get the point in camera coordinate
				point_c = Vector4f(x, y, z, 1.0);

                //get the point in global coordinate/ world coordinate
                point_w =  _trajectory.inverse() * _depthExtrinsics.inverse() * point_c;
                

            }
            vertices[idx].color = color;
            vertices[idx].position = point_w;
        }
    }

    //compute normal, apply Principal Component Analysis
    //1. search for points in the neighbourhood
    //2. compute principal component
    //3. normalize the norm
    for(unsigned int row = 1; row < _height - 1; row++){
        for(unsigned int col = 1; col < _width - 1; col++){
            int idx = row * _width + col;

            //1. search for points in the neighbourhood
            Vector4f point = vertices[idx].position;
            Vector4f leftPoint = vertices[idx - 1].position;
            Vector4f rightPoint = vertices[idx + 1].position;
            Vector4f upperPoint = vertices[idx - _width].position;
            Vector4f lowerPoint = vertices[idx + _width].position;

            //2. compute principal component
            Vector4f du =  vertices[idx + 1].position - vertices[idx - 1].position;
            Vector4f dv =  vertices[idx + _width].position - vertices[idx - _width].position;

            //we set normal to invalid when vertex are too far away from its neigbours
            // if(!du.allFinite()||!dv.allFinite()||du.norm() >= _edgeThreshold||dv.norm() >= _edgeThreshold){
            if(du.norm() >= _edgeThreshold||dv.norm() >= _edgeThreshold){
                vertices[idx].normal = Vector3f(MINF, MINF, MINF);
            }
            else{
                // getting the norm by cross product of two vectors made up of neighbours
                Vector3f normal = cross(du,dv);
                normal = normal.normalized();

                //3. normalize the norm
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

Matrix4f Frame::getDepthExtrinsics(){
    return _depthExtrinsics;
}

Matrix3f Frame::getDepthIntrinsics(){
    return _depthIntrinsics;
}


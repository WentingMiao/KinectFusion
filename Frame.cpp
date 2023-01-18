#include "Frame.h"



//constructor of Frame
Frame::Frame(float* depthMap,  BYTE* colorMap, Eigen::Matrix3f &depthIntrinsics, Eigen::Matrix4f &depthExtrinsics, 
         Eigen::Matrix4f &trajectory, unsigned int width,unsigned int height, float edgeThreshold )
: _width(width), _height(height), _depthIntrinsics(depthIntrinsics), _depthExtrinsics(depthExtrinsics), _edgeThreshold(edgeThreshold),_trajectory(trajectory)
{
    _depthMap = vector<float>(_width * _height);
    _colorMap = vector<Vector4uc>(_width * _height);
    for (unsigned int i = 0 ; i < _width * _height; i++){
         _depthMap[i] = depthMap[i];

         // color is stored as RGBX in row major (4 byte values per pixel) 
         // so the size of colorMap is 4 * width * height
         // we convert it to 4 unsigned char type
         _colorMap[i] = Vector4uc(colorMap[4*i], colorMap[4*i+1], colorMap[4*i+2], colorMap[4*i+3]);
    }

    vector<float> filteredDepthMap =  vector<float>(_width * _height);
    applyBilateralFilter( _depthMap,  filteredDepthMap);


}


vector<Eigen::Vector4f> Frame::getCameraPoints(){
    
    vector<Eigen::Vector4f> points( _width*_height );

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
/*setter function*/
void Frame::setFilterSize(int size){
    _dValue = size;
}

void Frame::setSigmaColor(double sigmaColor){
    _sigmaColor = sigmaColor;
}

void Frame::setSigmaSpace(double sigmaSpace){
    _sigmaSpace = sigmaSpace;
}

/*getter function */

vector<Vertex>  Frame::getVertices(){

    vector<Vertex> vertices(_width * _height);

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

vector<float> Frame::getDepthMap(){
    return _depthMap;
}

vector<Vector4uc> Frame::getColorMap(){
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

void Frame::applyBilateralFilter(vector<float>& originalDepth, vector<float>& outputDepth){
    
    const Mat cvOriginalDepth(_width, _height,  CV_32F, reinterpret_cast<void*>(originalDepth.data()));

    Mat cvOutputDepth(_width, _height, CV_32F, reinterpret_cast<void*>(outputDepth.data()));


    constexpr float BIG_NEGATIVE_NUMBER = -10000.0;

    
    for (size_t i = 0; i < originalDepth.size(); i++) {
        if (originalDepth[i] == MINF) {
            originalDepth[i] = BIG_NEGATIVE_NUMBER;
        }
    }

    bilateralFilter(cvOriginalDepth, cvOutputDepth, _dValue, _sigmaColor, _sigmaSpace);
    for (size_t i = 0; i < originalDepth.size(); i++) {
        if (originalDepth[i] == BIG_NEGATIVE_NUMBER) {
            originalDepth[i] = MINF;
            outputDepth[i] = MINF;
        }
    }

    
}

bool ValidFace(vector<Vertex>& vertices, unsigned int v1, unsigned int v2, unsigned int v3, float edgeThreshold){
	if(vertices[v1].position(0)== MINF || vertices[v2].position(0)== MINF ||vertices[v3].position(0)== MINF) return false;
	if(((vertices[v1].position - vertices[v2].position).norm() < edgeThreshold) && 
		((vertices[v1].position - vertices[v3].position).norm() < edgeThreshold) &&
		((vertices[v2].position - vertices[ v3].position).norm() < edgeThreshold)){
			return true;
		}
	else{
		return false;
	}
}

bool Frame::writeMesh(vector<Vertex>& vertices, const string& filename){

    float edgeThreshold = 0.01f; // 1cm

    unsigned int nVertices = _width * _height;

    unsigned nFaces = 0;	//Determine number of valid faces

    
	vector<Vector3i> faces;

	
	for(int row=0; row < _height-1; row++){
		for(int col=0; col < _width-1; col++){
			int id1 = row * _width + col;
			int id2 = row * _width + col + 1;
			int id3 = (row + 1) * _width +col;
			int id4 = (row + 1) * _width +col + 1;

			if(ValidFace(vertices,id1,id2,id3,edgeThreshold)){
				faces.push_back(Vector3i(id1,id2,id3));
	 			nFaces++;
			}

			if(ValidFace(vertices, id2, id3, id4, edgeThreshold)){
				faces.push_back(Vector3i(id2,id3,id4));
	 			nFaces++;
			}
		}
	}


    // Write off file
	ofstream outFile(filename);

    if (!outFile.is_open()) return false;
	
	
    outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

    outFile << "# list of vertices" << std::endl;

	outFile << "# X Y Z R G B A" << std::endl;

    //save vertices
    for(auto idx = 0; idx < _height * _width ; ++idx){
		if(vertices[idx].position.x() == MINF){
			outFile << 0.0 << " " << 0.0 << " " << 0.0 << " "
                     << 255 << " " << 255 << " " << 255 << " " << 255 << endl;
		}
		else{
			outFile << vertices[idx].position.x() 				  << " " 
					<< vertices[idx].position.y() 				  << " " 
					<< vertices[idx].position.z() 				  << " " 
					<< (unsigned int)vertices[idx].color[0] 	  << " "
					<< (unsigned int)vertices[idx].color[1] 	  << " "
					<< (unsigned int)vertices[idx].color[2] 	  << " "
					<< (unsigned int)vertices[idx].color[3] 	  
					<< endl;

					
		}
	}

    outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;

    //save valid faces
    for(int i = 0; i<faces.size();i++){
		outFile << 3 <<" "<< faces[i][0]<<" "<<faces[i][1] <<" "<<faces[i][2]<<std::endl;
	}

    // close file
	outFile.close();

	return true;



    return true;

}

void Frame::buildDepthPyramid(vector<float>& originalMap, vector<vector<float>>& outputMap, unsigned int maxLevel){

    const Mat cvOriginalMap(_width, _height,  CV_32F, reinterpret_cast<void*>(originalMap.data()));

    vector<Mat> gpyramid;

    buildPyramid(cvOriginalMap, gpyramid, maxLevel);

    /* convert mat to vector */
    for(size_t i = 0; i < gpyramid.size(); i++){
        vector<float> arr;
        if (gpyramid[i].isContinuous()) {
            arr.assign((float*)gpyramid[i].data, (float*)gpyramid[i].data + gpyramid[i].total()*gpyramid[i].channels());
        }else{
            for (int i = 0; i < gpyramid[i].rows; ++i) {
              arr.insert(arr.end(), gpyramid[i].ptr<float>(i), gpyramid[i].ptr<float>(i) + gpyramid[i].cols*gpyramid[i].channels());
            }
        }
        outputMap.push_back(arr);
    }

}

void Frame::buildColorPyramid(vector<Vector4uc>& originalMap, vector<vector<Vector4uc>>& outputMap, unsigned int maxLevel){
    //todo

}


Matrix3f Frame::getLevelCameraIntrinstics(unsigned int level){
    if(level == 0){
        return _depthIntrinsics;
    }

    Matrix3f levelCameraIntrinstics{_depthIntrinsics};

    float scale = pow(0.5, level);

    levelCameraIntrinstics(0,0) *= scale; // focal x
    levelCameraIntrinstics(1,1) *= scale; // focal y

    levelCameraIntrinstics(0,1) *= scale;  //axis skew (usually 0)

    levelCameraIntrinstics(0,2) *= scale; //principal point mx
    levelCameraIntrinstics(1,2) *= scale; // principal point my

    return levelCameraIntrinstics;
}
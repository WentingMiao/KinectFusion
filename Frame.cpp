#include "Frame.h"


// Frame::Frmae( float* depthMap,  BYTE* colorMap, Matrix3f depthIntrinsics, Matrix4f depthExtrinsicsInv, unsigned int width, unsigned int height)
Frame::Frame(VirtualSensor sensor, float edgeThreshold)
    : width(sensor.GetDepthImageWidth()), height(sensor.GetDepthImageHeight()), depthMap(sensor.GetDepth()), colorMap(sensor.GetColorRGBX()), depthIntrinsics(sensor.GetDepthIntrinsics()),edgeThreshold(edgeThreshold) ,depthExtrinsics(sensor.GetDepthExtrinsics()), trajectory(sensor.GetTrajectory()){
        //  std::cout << depthMap[20] <<std::endl;
        depthMap = sensor.GetDepth();
    }

std::vector<Eigen::Vector4f> Frame::fromPixelToWorld(){
    
    std::vector<Eigen::Vector4f> points( width*height );

    float fX = depthIntrinsics(0, 0);
	float fY = depthIntrinsics(1, 1);
	float cX = depthIntrinsics(0, 2);
	float cY = depthIntrinsics(1, 2);

    for(unsigned int row = 0; row < height; row++){
        for(unsigned int col = 0; col < width; col++){
            int idx = row * width + col;
            float z = depthMap[idx]; 
            
            if(z == MINF){
                points[idx] = Vector4f(MINF, MINF, MINF, MINF);
			}else{
                float u = col;
				float v = row;
				float x = z*(u-cX)/fX; 
				float y = z*(v-cX)/fY; 
                
				// point_c = Vector4f(x, y, x, 1.0);
                points[idx] =  trajectory.inverse() * depthExtrinsics.inverse() * Vector4f(x, y, x, 1.0);
                
            }
        }
    }
    return points;

}

Vector3f Frame::cross(Vector4f v1, Vector4f v2){
    Vector3f v1_ = v1.head<3>();
    Vector3f v2_ = v2.head<3>();
    return v1_.cross(v2_);
}

std::vector<Vertex>  Frame::getVertices(){

    std::vector<Vertex> vertices(width * height);

    /* get the 3d position and color */
    float fX = depthIntrinsics(0, 0);
	float fY = depthIntrinsics(1, 1);
	float cX = depthIntrinsics(0, 2);
	float cY = depthIntrinsics(1, 2);

    for(unsigned int row = 0; row < height; row++){
        for(unsigned int col = 0; col < width; col++){
            int idx = row * width + col;
            float z = depthMap[idx]; 
            Vector4f point_c, point_w;
            Vector4uc color = Vector4uc(colorMap[4*idx], colorMap[4*idx+1], colorMap[4*idx+2], colorMap[4*idx+3]);
			

            if(z == MINF){
                point_w = Vector4f(MINF, MINF, MINF, MINF);
                color = Vector4uc(0,0,0,0);
			}else{
                float u = col;
				float v = row;

				float x = z*(u-cX)/fX; 
				float y = z*(v-cX)/fY; 
                
				point_c = Vector4f(x, y, z, 1.0);
                point_w =  trajectory.inverse() * depthExtrinsics.inverse() * point_c;
                

            }
            vertices[idx].color = color;
            vertices[idx].position = point_w;
        }
    }

    //compute normal

    for(unsigned int row = 1; row < height - 1; row++){
        for(unsigned int col = 1; col < width - 1; col++){
            int idx = row * width + col;
            // int idxRight = row * width + ( col + 1);
            // int idxLeft = row * width + ( col - 1);
            // int idxUpper = ( row - 1) * width + col;
            // int idxLower = ( row + 1) * width + col;

            Vector4f point = vertices[idx].position;
            Vector4f leftPoint = vertices[idx - 1].position;
            Vector4f rightPoint = vertices[idx + 1].position;
            Vector4f upperPoint = vertices[idx - width].position;
            Vector4f lowerPoint = vertices[idx + width].position;


            Vector4f du =  vertices[idx + 1].position - vertices[idx - 1].position;
            Vector4f dv =  vertices[idx + width].position - vertices[idx - width].position;

            //we set normal to invalid when vertex are too far away from its neigbours
            if(!du.allFinite()||!dv.allFinite()||du.norm() >= edgeThreshold||dv.norm() >= edgeThreshold){
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
    for (int u = 0; u < width; ++u) {
        vertices[u].normal = Vector3f(MINF, MINF, MINF);
        vertices[u + (height - 1) * width].normal = Vector3f(MINF, MINF, MINF);
    }

    for (int v = 0; v < height; ++v) {
        vertices[v * width].normal = Vector3f(MINF, MINF, MINF);
        vertices[(width - 1) + v * width].normal = Vector3f(MINF, MINF, MINF);
    }


    return vertices;


}
// std::vector<Eigen::Vector4f> fromCameraToWorld(std::vector<Eigen::Vector4f> points_c){
    
// }





float* Frame::getDepthMap(){

    return depthMap;
}
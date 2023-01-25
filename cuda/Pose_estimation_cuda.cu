#include "Pose_estimation_cuda.h"

namespace kinectfusion{
    
__device__
Vector3f Vector4fToVector3f(Vector4f vertex){
    Vector3f output;
    output.x() = vertex.x();
    output.y() = vertex.y();
    output.z() = vertex.z();
    return output;
}
__device__
Vector4f Vector3fToVector4f(Vector3f vertex){
    Vector4f output;
    output.x() = vertex.x();
    output.y() = vertex.y();
    output.z() = vertex.z();
    output.w() = 1.0;
    return output;
}

__device__
Vector3f TransformToVertex(Vector3f vertex, Eigen::Matrix4f Transformation)
{
    return Transformation.block(0, 0, 3, 3) * vertex + Transformation.block(0,3,3,1);

}

__device__
Vector3f TransformToNormal(Vector3f normal, Eigen::Matrix4f Transformation)
{
    return Transformation.block(0, 0, 3, 3) * normal;

}

__global__ 
void data_association_kernel(           const Vertex* frame_data,
                                        const Matrix3f* Intrinsics,
                                        const unsigned int width,
                                        const unsigned int height,
                                        Match* matches,
                                        unsigned int* match_count,
                                        unsigned int frame_data_size,
                                        const Matrix4f previous_pose,
                                        const Matrix4f current_pose)
{
    float fX = Intrinsics[0](0,0);
    float fY = Intrinsics[0](1,1);
    float cX = Intrinsics[0](0,2);
    float cY = Intrinsics[0](1,2);
    
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    // const int y = blockIdx.y * blockDim.y + threadIdx.y;
    // printf("Thread %d found a match\n", idx); 
    // int idx = y * width + x;
    if (idx >= frame_data_size) {
        return;
    }

    Vector3f current_vertex = Vector4fToVector3f(frame_data[idx].position);
    Vector3f current_normal = frame_data[idx].normal;

    // avoid redundant caculation to speed up 
    if (!isnan(current_normal[0]) && !isnan(current_normal[1]) && !isnan(current_normal[2]) &&
        !isnan(current_vertex[0]) && !isnan(current_vertex[0]) && !isnan(current_vertex[0]) &&
        current_normal[0] != MINF && current_normal[2] != MINF && current_normal[2] != MINF &&
        current_vertex[0] != MINF && current_vertex[2] != MINF && current_vertex[2] != MINF)
    {
        // transfrom to camera coordinate v_k-1
        Eigen::MatrixXf tmpTransformation = previous_pose.inverse() * current_pose;
        Eigen::Vector3f current_camera_vertex = TransformToVertex(current_vertex, tmpTransformation);            
            // back-project to pixel v_k-1
        Eigen::Vector2i point;
        point.x() = std::round(current_camera_vertex.x() * fX / current_camera_vertex.z() + cX);
        point.y() = std::round(current_camera_vertex.y() * fY / current_camera_vertex.z() + cY);

        // check if pixel still in image
        // && current_camera_vertex.z() >= 0
        if(point.x() >= 0 && point.y() >= 0 && point.x() < width && point.y() < height && current_camera_vertex.z() >= 0){
            //cacluate v
            int previous_idx = point.y() * width + point.x();
            // i means point in frame k
            // previous means point in frame k-1

            // printf("Thread %d found a match\n", idx);
    
            int match_index = atomicAdd(match_count, 1);
            matches[match_index].cur_idx = idx;
            matches[match_index].prv_idx = previous_idx;            

        }                              
    }
}
    
void data_association_cuda(     const std::vector<Vertex>& frame_data,
                                const Matrix3f& Intrinsics,
                                const unsigned int& width,
                                const unsigned int& height,
                                std::unordered_map<int, int>& matches,
                                const Eigen::MatrixXf& previous_pose,
                                const Eigen::MatrixXf& current_pose
                                )
{
    // Allocate memory on the GPU + Copy data from host to device
    Vertex* d_frame_data;
    cudaMalloc(&d_frame_data, sizeof(Vertex) * frame_data.size());
    cudaMemcpy(d_frame_data, frame_data.data(), sizeof(Vertex) * frame_data.size(), cudaMemcpyHostToDevice);
    
    Matrix3f* d_Intrinsics;
    cudaMalloc(&d_Intrinsics, sizeof(Matrix3f));
    cudaMemcpy(d_Intrinsics, &Intrinsics, sizeof(Matrix3f), cudaMemcpyHostToDevice);

    Match* d_matches;
    unsigned int* d_match_count;
    cudaMalloc(&d_matches, frame_data.size() * sizeof(Match));
    cudaMalloc(&d_match_count, sizeof(unsigned int));
    int match_count = 0;
    cudaMemcpy(d_match_count, &match_count, sizeof(unsigned int), cudaMemcpyHostToDevice);    

    // // test memcpy
    // std::vector<Vertex> h_frame_data(frame_data.size());
    // cudaMemcpy(h_frame_data.data(), d_frame_data, sizeof(Vertex) * frame_data.size(), cudaMemcpyDeviceToHost);
    
    // for(int i=0;i<frame_data.size();i++)
    //     std::cout<<h_frame_data[i].position<<std::endl;    
    
    // Launch the kernel
    dim3 threads(256);
    dim3 blocks((frame_data.size() + threads.x - 1) / threads.x);    

    data_association_kernel <<<blocks, threads>>> (d_frame_data, d_Intrinsics, width, height, d_matches, d_match_count, frame_data.size(), previous_pose, current_pose);

    cudaError_t error = cudaGetLastError();
    if(error != cudaSuccess)
    {
        // print the CUDA error message and exit
        printf("CUDA error: %s\n", cudaGetErrorString(error));
        exit(-1);
    }

    cudaDeviceSynchronize();
    // Copy data from device to host

    cudaMemcpy(&match_count, d_match_count, sizeof(int), cudaMemcpyDeviceToHost);

    Match* temp_matches_gpu = new Match[match_count];
    cudaMemcpy(temp_matches_gpu, d_matches, match_count * sizeof(Match), cudaMemcpyDeviceToHost);

    matches.clear();
    for (int i = 0; i < match_count; i++) {
        matches.insert({temp_matches_gpu[i].cur_idx, temp_matches_gpu[i].prv_idx});
    }

    delete[] temp_matches_gpu;

    // cout << "value of match_count " << match_count << endl;

    // Free the GPU memory

    cudaFree(d_frame_data);
    cudaFree(d_matches);
    cudaFree(d_match_count);
    cudaFree(d_Intrinsics);
}

}
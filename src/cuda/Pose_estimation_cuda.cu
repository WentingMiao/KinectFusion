#include "Pose_estimation_cuda.h"
#define PI acos(-1)

#define HANDLE_ERROR(err) (HandleError(err, __FILE__, __LINE__))

static void HandleError(cudaError_t err, const char *file, int line) {
	if (err != cudaSuccess) {
		fprintf(stderr, "Error %d: \"%s\" in %s at line %d\n", int(err), cudaGetErrorString(err), file, line);
		exit(int(err));
	}
}


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
                                        const Matrix4f current_pose,
                                        const Matrix4f previous_pose,
                                        const Vertex* model_data,
                                        const float distance_threshold,
                                        const float angle_threshold,
                                        double* loss                                        
                                        )
{
    float fX = Intrinsics[0](0,0);
    float fY = Intrinsics[0](1,1);
    float cX = Intrinsics[0](0,2);
    float cY = Intrinsics[0](1,2);
    
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    // printf("Thread %d found a match\n", idx); 
    int idx = y * width + x;
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

            Vector3f current_global_vertex = TransformToVertex(Vector4fToVector3f(frame_data[idx].position), current_pose);
            Vector3f current_global_normal = TransformToNormal(frame_data[idx].normal, current_pose); 
            Vector3f previous_global_vertex = TransformToVertex(Vector4fToVector3f(model_data[previous_idx].position), previous_pose);
            Vector3f previous_global_normal = TransformToNormal(model_data[previous_idx].normal, previous_pose);    

            if (!isnan(current_global_vertex[0]) && !isnan(current_global_vertex[1]) && !isnan(current_global_vertex[2]) &&
                !isnan(previous_global_vertex[0]) && !isnan(previous_global_vertex[1]) && !isnan(previous_global_vertex[2]) &&
                current_global_vertex[0] != MINF && current_global_vertex[1] != MINF && current_global_vertex[2] != MINF &&
                previous_global_vertex[0] != MINF && previous_global_vertex[1] != MINF && previous_global_vertex[2] != MINF
                && !isnan(previous_global_normal[0]) && !isnan(previous_global_normal[1]) && !isnan(previous_global_normal[2]) &&
                previous_global_normal[0] != MINF && previous_global_normal[1] != MINF && previous_global_normal[2] != MINF){

                //caculate norm
                const float distance = (previous_global_vertex - current_global_vertex).norm();
                
                if (distance <= distance_threshold){

                    auto normal_angle = acos(previous_global_normal.dot(current_global_normal) / 
                    (previous_global_normal.norm() * current_global_normal.norm()));
                    normal_angle = normal_angle * 180/PI;
                    if(normal_angle < angle_threshold){
                        int match_index = atomicAdd(match_count, 1);
                        matches[match_index].cur_idx = idx;
                        matches[match_index].prv_idx = previous_idx;  
                        atomicAdd(loss,distance);
                    }
                }
            }
        }                              
    }
}


__global__ 
void incremental_caculation_kernel (    const Vertex* frame_data,
                                        const Vertex* model_data,
                                        Match* matches,
                                        unsigned int* match_count,
                                        const Matrix4f previous_pose,
                                        const Matrix4f current_pose,                                        
                                        Matrix<float, 1,6>* A,
                                        float* b,
                                        unsigned int* matrix_count
                                )
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx >= *match_count) {
        return;
    }
    
    auto source_idx = matches[idx].cur_idx;
    auto target_idx = matches[idx].prv_idx;             
    // printf("source_idx %d ,targtet_idx %d\n", source_idx,target_idx);                                
    // Note that sourcePoint always in camera space
    Vector3f s  = TransformToVertex(Vector4fToVector3f(frame_data[source_idx].position),current_pose);
    Vector3f d  = TransformToVertex(Vector4fToVector3f(model_data[target_idx].position),previous_pose);
    Vector3f n  = TransformToNormal(model_data[target_idx].normal,previous_pose);

    Matrix<float, 1, 6> point2plane_A; 
    point2plane_A << n(2)*s(1)-n(1)*s(2), n(0)*s(2)-n(2)*s(0), n(1)*s(0)-n(0)*s(1), n(0), n(1), n(2);
    A[idx] = point2plane_A;
    float point2plane_b = (n(0)*d(0) + n(1)*d(1) + n(2)*d(2) - n(0)*s(0) - n(1)*s(1) - n(2)*s(2));
    b[idx] = point2plane_b;

    atomicAdd(matrix_count, 1);

}


std::tuple<MatrixXf, VectorXf> data_association_cuda(     const std::vector<Vertex>& frame_data,
                                const Matrix3f& Intrinsics,
                                const unsigned int& width,
                                const unsigned int& height,
                                std::unordered_map<int, int>& matches,
                                const Eigen::MatrixXf& previous_pose,
                                const Eigen::MatrixXf& current_pose,
                                const std::vector<Vertex>& model_data,
                                const float& distance_threshold,
                                const float& angle_threshold,
                                double& loss
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
    cudaMalloc(&d_matches, frame_data.size() * sizeof(Match));

    unsigned int* d_match_count;
    cudaMalloc(&d_match_count, sizeof(unsigned int));
    int match_count = 0;
    cudaMemcpy(d_match_count, &match_count, sizeof(unsigned int), cudaMemcpyHostToDevice);    

    Vertex* d_model_data;
    cudaMalloc(&d_model_data, sizeof(Vertex) * model_data.size());
    cudaMemcpy(d_model_data, model_data.data(), sizeof(Vertex) * model_data.size(), cudaMemcpyHostToDevice);

    double *d_loss;
    cudaMalloc(&d_loss, sizeof(double));
    cudaMemcpy(d_loss, &loss, sizeof(double), cudaMemcpyHostToDevice);
    // // test memcpy
    // std::vector<Vertex> h_frame_data(frame_data.size());
    // cudaMemcpy(h_frame_data.data(), d_frame_data, sizeof(Vertex) * frame_data.size(), cudaMemcpyDeviceToHost);
    
    // for(int i=0;i<frame_data.size();i++)
    //     std::cout<<h_frame_data[i].position<<std::endl;    
    
    // Launch the kernel
    // dim3 threads(256);
    // dim3 blocks((frame_data.size() + threads.x - 1) / threads.x);    
    dim3 block(32,32);
    dim3 grid(  (width + block.x - 1) / block.x,
                (height + block.y - 1) / block.y);    
    data_association_kernel <<<grid, block>>> ( d_frame_data, 
                                                d_Intrinsics, 
                                                width, 
                                                height, 
                                                d_matches, 
                                                d_match_count, 
                                                frame_data.size(), 
                                                current_pose, 
                                                previous_pose,
                                                d_model_data, 
                                                distance_threshold, 
                                                angle_threshold, 
                                                d_loss);


    HANDLE_ERROR(cudaDeviceSynchronize());

    cudaMemcpy(&match_count, d_match_count, sizeof(int), cudaMemcpyDeviceToHost);

    // handle matches to test effect
    Match* temp_matches_gpu = new Match[match_count];
    cudaMemcpy(temp_matches_gpu, d_matches, match_count * sizeof(Match), cudaMemcpyDeviceToHost);

    matches.clear();
    for (int i = 0; i < match_count; i++) {
        matches.insert({temp_matches_gpu[i].cur_idx, temp_matches_gpu[i].prv_idx});
    }

    cudaMemcpy(&loss, d_loss, sizeof(double), cudaMemcpyDeviceToHost);

    delete[] temp_matches_gpu;

    
    Matrix<float, 1, 6> *h_A = new Matrix<float, 1,6>[match_count];
    float *h_b = new float[match_count];

    Matrix<float, 1,6>* d_A;
    cudaMalloc((void **)&d_A, sizeof(Matrix<float, 1, 6>) * match_count);
    float* d_b;
    cudaMalloc((void **)&d_b, sizeof(float) * match_count);   

    unsigned int* d_matrix_count;
    cudaMalloc(&d_matrix_count, sizeof(unsigned int));
    int matrix_count = 0;
    cudaMemcpy(d_matrix_count, &matrix_count, sizeof(unsigned int), cudaMemcpyHostToDevice);     
    // incremental_caculation 
    dim3 threads(256);
    dim3 blocks((match_count + threads.x - 1) / threads.x);  
    
    incremental_caculation_kernel <<<blocks, threads>>> (   d_frame_data, 
                                                            d_model_data, 
                                                            d_matches,
                                                            d_match_count, 
                                                            previous_pose, 
                                                            current_pose,
                                                            d_A, 
                                                            d_b, 
                                                            d_matrix_count);

    HANDLE_ERROR(cudaDeviceSynchronize());

    cudaMemcpy(h_A, d_A, sizeof(Matrix<float, 1, 6>) * match_count, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_b, d_b, sizeof(float) * match_count, cudaMemcpyDeviceToHost);   

    MatrixXf A = MatrixXf::Zero(match_count, 6);
    VectorXf b = VectorXf::Zero(match_count);
    for(int i = 0; i < match_count; i++)
        {
            A.row(i) = h_A[i];
            b(i) = h_b[i];
        }



    
    // Free the GPU memory

    cudaFree(d_frame_data);
    cudaFree(d_matches);
    cudaFree(d_match_count);
    cudaFree(d_Intrinsics);
    cudaFree(d_model_data);
    cudaFree(d_loss);
    cudaFree(d_A);
    cudaFree(d_b);
    cudaFree(d_matrix_count);
    return std::tuple(A,b);
}



}
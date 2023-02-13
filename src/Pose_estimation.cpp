#include "Pose_estimation.h"

#define PI acos(-1)
std::mutex mut;


bool Pose::pose_estimation(const std::vector<vector<Vertex>>& frame_data,
                     const std::vector<vector<Vertex>>& model_data,
                     const std::vector<Matrix3f>& intrinstics,
                     const float distance_threshold,
                     const float angle_threshold, //with angle °
                     const std::vector<int>& num_iteration,
                     const std::vector<int>& width,
                     const std::vector<int>& height,
                     const int& pyramid_level,
                     Eigen::Matrix4f& cur_pose
)
{
    clock_t main_begin = clock();
    // step 1: Initialize pose 
    m_current_pose = cur_pose;
    m_previous_pose = m_current_pose;
    


    for(auto level = pyramid_level -1; level >= 0; --level){
        
        // step 2: Data association

        // Note that we use current points in k project to k-1, but still can find correspondence

        for(int it = 0; it < num_iteration[level]; ++it){
            
            clock_t begin = clock();

            // metrics to test the performance, we choose MAE(Mean average error)
            double error = 0.0; 

            // map to store the matches pair 
            std::unordered_map<int, int> matches;
            
            // step 2.1: data association + back projection
            // step 2.2: outlier check 
            // step 2.3: build linear system
            // all caculation turn to cuda
            std::tuple<MatrixXf, VectorXf> lgs = kinectfusion::data_association_cuda(frame_data[level],
                                                                                     intrinstics[level], 
                                                                                     width[level], 
                                                                                     height[level], 
                                                                                     matches, 
                                                                                     m_previous_pose, 
                                                                                     m_current_pose,
                                                                                     model_data[level], 
                                                                                     distance_threshold, 
                                                                                     angle_threshold, 
                                                                                     error);

            MatrixXf A = std::get<0>(lgs);
            VectorXf b = std::get<1>(lgs);
            // step 2.3: sovle linear system --> get pose
            incremental_caculation(A, b);

            // for evaluation

            clock_t end = clock();
            double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;

        } 
    }

    if(DEBUG){
        clock_t main_end = clock();
        double elapsedSecs = double(main_end - main_begin) / CLOCKS_PER_SEC;
        std::cout << "Pose estimation finished in " << elapsedSecs << " seconds. " << std::endl;
    }

    cur_pose = m_current_pose;    
    return true;
}


void Pose::incremental_caculation   (MatrixXf& A,
                                     VectorXf& b)
{        
        MatrixXf ATA = A.transpose() * A;
        MatrixXf ATb = A.transpose() * b;
        VectorXf x(6);
        x = ATA.ldlt().solve(ATb);
        // x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
        // x = A.fullPivLu().solve(b);

        float alpha = x(0), beta = x(1), gamma = x(2);

        Matrix3f rotation_incremental = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
                                        AngleAxisf(beta,  Vector3f::UnitY()).toRotationMatrix() *
                                        AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();    

        Vector3f translation_incremental = x.tail(3); 
        Matrix4f pose_incremental = Matrix4f::Identity();
        pose_incremental.block<3,3>(0,0) = rotation_incremental;
        pose_incremental.block<3,1>(0,3) = translation_incremental;

        m_current_pose = pose_incremental * m_current_pose;            

        if(DEBUG){
            // cout << "incremental is: " << endl << pose_incremental << endl; 
            // cout << "new pose is: " << endl << m_current_pose << endl;
        }        
}



Vector3f Pose::TransformToVertex(Vector3f vertex, Eigen::Matrix4f Transformation)
{
    return Transformation.block(0, 0, 3, 3) * vertex + Transformation.block(0,3,3,1);

}

Vector3f Pose::TransformToNormal(Vector3f normal, Eigen::Matrix4f Transformation)
{
    return Transformation.block(0, 0, 3, 3) * normal;

}

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
            std::unordered_map<int, int> selected_matches;

            // step 2.1: data association + back projection
            data_association(frame_data[level], intrinstics[level], width[level], height[level], matches);

            // step 2.2: outlier check 
            outlier_check(frame_data[level], model_data[level], matches, selected_matches, distance_threshold, angle_threshold, error);

            // for evaluation

            clock_t end = clock();
            double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
            if(DEBUG){ 
                std::cout << endl << "level is: " << level << endl;
                std::cout << "current iteration is: " << it << endl;
                std::cout << "Data association completed in " << elapsedSecs << " seconds. " << std::endl;
                std::cout << "find match pairs: " << matches.size() << ", after detection: " << selected_matches.size() << std::endl;
                std::cout << "avarage error is " << error / selected_matches.size() << std::endl;
                if(!selected_matches.size()) throw std::out_of_range("No match pairs, some error exist!!");

            }

            // step 2.3: point to plane ICP
            incremental_caculation(frame_data[level], model_data[level], selected_matches);

        } 
    }

    if(DEBUG){
        std::cout << "Optimization iteration done " << std::endl;
    }


    cur_pose = m_current_pose;    
    return true;
}


void Pose::data_association(const std::vector<Vertex>& frame_data,
                            const Matrix3f &Intrinsics,
                            const unsigned int& width,
                            const unsigned int& height,
                            std::unordered_map<int, int>& matches)
{   
    float fX =    Intrinsics(0, 0);
    float fY =    Intrinsics(1, 1);
    float cX =    Intrinsics(0, 2);
    float cY =    Intrinsics(1, 2);

    for(int i = 0; i < frame_data.size(); ++i){

        // current_vertex: in camera space of k         
        Vector3f current_vertex = Vector4fToVector3f(frame_data[i].position);
        Vector3f current_normal = frame_data[i].normal;

        // avoid redundant caculation to speed up 
        if (!isnan(current_normal[0]) && !isnan(current_normal[1]) && !isnan(current_normal[2]) &&
            !isnan(current_vertex[0]) && !isnan(current_vertex[0]) && !isnan(current_vertex[0]) &&
            current_normal[0] != MINF && current_normal[2] != MINF && current_normal[2] != MINF &&
            current_vertex[0] != MINF && current_vertex[2] != MINF && current_vertex[2] != MINF){
        
            // transfrom to camera coordinate v_k-1
            Eigen::MatrixXf tmpTransformation = m_previous_pose.inverse() * m_current_pose;
            Eigen::Vector3f current_camera_vertex = TransformToVertex(current_vertex, tmpTransformation);
            
            // back-project to pixel v_k-1
            Eigen::Vector2i point;
            point.x() = std::round(current_camera_vertex.x() * fX / current_camera_vertex.z() + cX);
            point.y() = std::round(current_camera_vertex.y() * fY / current_camera_vertex.z() + cY);

            // check if pixel still in image
            // && current_camera_vertex.z() >= 0
            if(point.x() >= 0 && point.y() >= 0 && point.x() < width && point.y() < height && current_camera_vertex.z() >= 0){
                //cacluate v
                unsigned int previous_idx = point.y() * width + point.x();
                // i means point in frame k
                // previous means point in frame k-1
                matches.insert(std::pair<int, int>(i, previous_idx));

                if(DEBUG){
                    // std::cout << "fX is: " << fX << " fY is: " << fY << endl;
                    // std::cout << "index in k: " << i << endl;
                    // // 当前帧的坐标是？
                    // int tmp_x = i % width;
                    // int tmp_y = i / width;
                    
                    // std::cout << "idex in k but with x and y:" << tmp_x << ", " << tmp_y << endl;                          
                    // std::cout << "current_vertex in (k camera space): " << endl << current_vertex << endl;

                    // std::cout << "current_camera_vertex in k-1 (camera space) : " << endl << current_vertex << endl;
                    // std::cout << "point: " << point.x() << ", " << point.y() << endl;      
                    // std::cout << "index in k-1: " << previous_idx << endl << endl;
                }
            }
        }
    }
}

void Pose::outlier_check(const std::vector<Vertex>& frame_data,
                                     const std::vector<Vertex>& model_data,
                                     std::unordered_map<int, int>& matches,
                                     std::unordered_map<int, int>& selected_matches,
                                     const float& distance_threshold,
                                     const float& angle_threshold,
                                     double& error)
{

    // version 1: with parallel

    std::for_each(std::execution::par_unseq, matches.begin(), matches.end(), [&](const auto& m) {

        int cur_idx = m.first;
        int prv_idx = m.second;

        Vector3f current_global_vertex = TransformToVertex(Vector4fToVector3f(frame_data[cur_idx].position), m_current_pose);
        Vector3f current_global_normal = TransformToNormal(frame_data[cur_idx].normal, m_current_pose); 
        Vector3f previous_global_vertex = TransformToVertex(Vector4fToVector3f(model_data[prv_idx].position), m_previous_pose);
        Vector3f previous_global_normal = TransformToNormal(model_data[prv_idx].normal, m_previous_pose);

        // avoid redundant caculation to speed up 
        if (!isnan(current_global_vertex[0]) && !isnan(current_global_vertex[1]) && !isnan(current_global_vertex[2]) &&
            !isnan(previous_global_vertex[0]) && !isnan(previous_global_vertex[1]) && !isnan(previous_global_vertex[2]) &&
            current_global_vertex[0] != MINF && current_global_vertex[1] != MINF && current_global_vertex[2] != MINF &&
            previous_global_vertex[0] != MINF && previous_global_vertex[1] != MINF && previous_global_vertex[2] != MINF &&
            !isnan(previous_global_normal[0]) && !isnan(previous_global_normal[1]) && !isnan(previous_global_normal[2]) &&
            previous_global_normal[0] != MINF && previous_global_normal[1] != MINF && previous_global_normal[2] != MINF){

            //caculate norm
            const float distance = (previous_global_vertex - current_global_vertex).norm();
            
            // std::cout << distance << std::endl;
            if (distance <= distance_threshold){

                auto normal_angle = acos(previous_global_normal.dot(current_global_normal) / 
                (previous_global_normal.norm() * current_global_normal.norm()));
                normal_angle = normal_angle * 180/PI;
                if(normal_angle < angle_threshold){
                    std::lock_guard<std::mutex> guard(mut);
                    selected_matches.insert(std::pair<int, int>(cur_idx, prv_idx));  
                    error += distance;
                }
            }
        }
    });    

    // version 2: without parallel

    // for(auto it = matches.begin(); it != matches.end(); ++it){

    //     int cur_idx = it->first;
    //     int prv_idx = it->second;

    //     Vector3f current_global_vertex = TransformToVertex(Vector4fToVector3f(frame_data[cur_idx].position), m_current_pose);
    //     Vector3f current_global_normal = TransformToNormal(frame_data[cur_idx].normal, m_current_pose); 
    //     Vector3f previous_global_vertex = TransformToVertex(Vector4fToVector3f(model_data[prv_idx].position), m_previous_pose);
    //     Vector3f previous_global_normal = TransformToNormal(model_data[prv_idx].normal, m_previous_pose);

    //     // avoid redundant caculation to speed up 
    //     if (!isnan(current_global_vertex[0]) && !isnan(current_global_vertex[1]) && !isnan(current_global_vertex[2]) &&
    //         !isnan(previous_global_vertex[0]) && !isnan(previous_global_vertex[1]) && !isnan(previous_global_vertex[2]) &&
    //         current_global_vertex[0] != MINF && current_global_vertex[1] != MINF && current_global_vertex[2] != MINF &&
    //         previous_global_vertex[0] != MINF && previous_global_vertex[1] != MINF && previous_global_vertex[2] != MINF
    //         && !isnan(previous_global_normal[0]) && !isnan(previous_global_normal[1]) && !isnan(previous_global_normal[2]) &&
    //         previous_global_normal[0] != MINF && previous_global_normal[1] != MINF && previous_global_normal[2] != MINF){

    //         //caculate norm
    //         const float distance = (previous_global_vertex - current_global_vertex).norm();
            
    //         // std::cout << distance << std::endl;
    //         if (distance <= distance_threshold){

    //             auto normal_angle = acos(previous_global_normal.dot(current_global_normal) / 
    //             (previous_global_normal.norm() * current_global_normal.norm()));
    //             normal_angle = normal_angle * 180/PI;
    //             if(normal_angle < angle_threshold){
    //                 // std::lock_guard<std::mutex> guard(mut);
    //                 selected_matches.insert(std::pair<int, int>(cur_idx, prv_idx));  
    //                 error += distance;
    //             }
    //         }
    //     }
    // }    
}

void Pose::incremental_caculation   (const std::vector<Vertex>& frame_data,
                                     const std::vector<Vertex>& model_data,
                                     std::unordered_map<int, int>& selected_matches)
{
        //step 2.2 ICP

        // if(DEBUG){
        // cout << "initial pose is :" << endl << m_current_pose << endl;
        // }        

        // prepare data for ICP

        // for parallel we consider caculate each point pair

        Vector3f s; // sourcePoint
        Vector3f d; // targetPoint
        Vector3f n;// targetNormal
        const size_t N = selected_matches.size();
        MatrixXf A = MatrixXf::Zero(N, 6);
        VectorXf b = VectorXf::Zero(N);
        int i = 0;

        // version 1: with parallel

        std::for_each(std::execution::par_unseq, selected_matches.begin(), selected_matches.end(), [&](const auto& it){
            auto source_idx = it.first;
            auto target_idx = it.second;            
            // Note that sourcePoint always in camera space
            s  = TransformToVertex(Vector4fToVector3f(frame_data[source_idx].position),m_current_pose);
            d  = TransformToVertex(Vector4fToVector3f(model_data[target_idx].position),m_previous_pose);
            n  = TransformToNormal(model_data[target_idx].normal,m_previous_pose);
            A.block<1, 3>(i, 0) = s.cross(n).transpose();
            A.block<1, 3>(i, 3) = n.transpose();
            // part of point2plane, copied from paper
            b(i) = (d - s).dot(n);
            ++i;
            // ICP incremental caculate
        });

        // version 2: without parallel

        // for(auto it = selected_matches.begin(); it != selected_matches.end(); ++it){
        //     auto source_idx = it->first;
        //     auto target_idx = it->second;            
        //     // Note that sourcePoint always in camera space
        //     s  = TransformToVertex(Vector4fToVector3f(frame_data[source_idx].position),m_current_pose);
        //     d  = TransformToVertex(Vector4fToVector3f(model_data[target_idx].position),m_previous_pose);
        //     n  = TransformToNormal(model_data[target_idx].normal,m_previous_pose);
        //     A.block<1, 3>(i, 0) = s.cross(n).transpose();
        //     A.block<1, 3>(i, 3) = n.transpose();
        //     // part of point2plane, copied from paper
        //     b(i) = (d - s).dot(n);
        //     ++i;
        //     // ICP incremental caculate
        // }

        
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


Vector3f Pose::Vector4fToVector3f(Vector4f vertex){
    Vector3f output;
    output.x() = vertex.x();
    output.y() = vertex.y();
    output.z() = vertex.z();
    return output;
}

Vector4f Pose::Vector3fToVector4f(Vector3f vertex){
    Vector4f output;
    output.x() = vertex.x();
    output.y() = vertex.y();
    output.z() = vertex.z();
    output.w() = 1.0;
    return output;
}

Vector3f Pose::TransformToVertex(Vector3f vertex, Eigen::Matrix4f Transformation)
{
    return Transformation.block(0, 0, 3, 3) * vertex + Transformation.block(0,3,3,1);

}

Vector3f Pose::TransformToNormal(Vector3f normal, Eigen::Matrix4f Transformation)
{
    return Transformation.block(0, 0, 3, 3) * normal;

}
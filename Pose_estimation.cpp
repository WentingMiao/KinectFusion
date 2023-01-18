#include "Pose_estimation.h"

#define PI acos(-1)
std::mutex mut;
/**
 * Convention: 
 * we focus on Rotation R and Translation t
 * and we always process pose from frame 1 to frame 2
 * R_w1, t_w1 means parameters in world coordinate of frame 1
 * R_c2, t_c2 means parametersin camera coordinate of frame 2
 */



bool Pose::pose_estimation(const std::vector<Vertex>& frame_data,
                     const std::vector<Vertex>& model_data,
                     const Matrix3f &Intrinsics,
                     const float distance_threshold,
                     const float angle_threshold, //with angle °
                     const int& num_iteration,
                     const unsigned int& width,
                     const unsigned int& height,
                     const unsigned int& pyramid_level,
                     Eigen::Matrix4f& cur_pose
)
{
    float fX =    Intrinsics(0, 0);
    float fY =    Intrinsics(1, 1);
    float cX =    Intrinsics(0, 2);
    float cY =    Intrinsics(1, 2);

    // double error = std::numeric_limits<double>::infinity();
    VectorXd pose_increment(6);

    // step 1: Initialize pose 
    Eigen::Matrix3f current_global_rotation    = cur_pose.block(0, 0, 3, 3); // R_w2 (before first iteration equal to R_w1)
    Eigen::Vector3f current_global_translation = cur_pose.block(0, 3, 3, 1); // t_w2 (before first iteration equal to R_w1)

    Eigen::Matrix3f previous_global_rotation_inverse(current_global_rotation.transpose()); // R_w1.T == R_w1^-1
    Eigen::Vector3f previous_global_translation = cur_pose.block(0, 3, 3, 1); // t_w1

    // step 2: ICP iterations

    for(int it = 0; it < num_iteration; ++it){
        // step 2.1: data association
        clock_t begin = clock();

        const unsigned nPoints = frame_data.size();
        std::unordered_map<int, int> matches;
        std::unordered_map<int, int> selected_matches;
        int count = 0;

        for(int i = 0; i < nPoints; ++i){

            // current_vertex: in camera space of k
            // previous_vertex: this vertex in camera space of k-1
            Vector3f current_vertex = Vector4fToVector3f(frame_data[i].position);
            // current_vertex.x() = frame_data[i].position.x();
            // current_vertex.y() = frame_data[i].position.y();
            // current_vertex.z() = frame_data[i].position.z();
            Vector3f current_normal = frame_data[i].normal;
            // get vertex in previous global vertex  V_g_k-1

            // avoid redundant caculation to speed up 
            if (!isnan(current_normal[0]) && !isnan(current_normal[1]) && !isnan(current_normal[2]) &&
                !isnan(current_vertex[0]) && !isnan(current_vertex[0]) && !isnan(current_vertex[0]) &&
                current_normal[0] != MINF && current_normal[2] != MINF && current_normal[2] != MINF &&
                current_vertex[0] != MINF && current_vertex[2] != MINF && current_vertex[2] != MINF){
                Eigen::Vector3f current_global_vertex = current_global_rotation * current_vertex + current_global_translation;
                // transfrom to camera coordinate v_k-1
                Eigen::Vector3f previous_vertex = previous_global_rotation_inverse * (current_global_vertex - previous_global_translation);            
 
                // back-project to pixel v_k-1
                Eigen::Vector2i point;
                point.x() = std::floor(previous_vertex.x() * fX / previous_vertex.z() + cX);
                point.y() = std::floor(previous_vertex.y() * fY / previous_vertex.z() + cY);

                // check if pixel still in image
                if(point.x() >= 0 && point.y() >= 0 && point.x() < width && point.y() < height && previous_vertex.z() >= 0){
                    //cacluate v
                    unsigned int previous_idx = point.y() * width + point.x();
                    // i means point in frame k
                    // previous means point in frame k-1
                    matches.insert(std::pair<int, int>(i, previous_idx));
                }
            }
        }

        std::for_each(std::execution::par_unseq, matches.begin(), matches.end(), [&](const auto& m) {

            int cur_idx = m.first;
            int prv_idx = m.second;
            Vector3f current_vertex = Vector4fToVector3f(frame_data[cur_idx].position);
            // current_vertex.x() = frame_data[cur_idx].position.x();
            // current_vertex.y() = frame_data[cur_idx].position.y();
            // current_vertex.z() = frame_data[cur_idx].position.z();
            Vector3f current_global_vertex = current_global_rotation * current_vertex + current_global_translation;
            
            Vector3f previous_global_vertex = Vector4fToVector3f(model_data[prv_idx].position);
            
            // previous_global_vertex.x() = model_data[prv_idx].position.x();
            // previous_global_vertex.y() = model_data[prv_idx].position.y();
            // previous_global_vertex.z() = model_data[prv_idx].position.z();
            Vector3f previous_global_normal = model_data[prv_idx].normal;

        
            const float distance = (previous_global_vertex - current_global_vertex).norm();
            if (distance < distance_threshold){
                //caculate norm
                //! normal and vertex of frame_data is in camera space
                //! normal and vertex of model_data is in world space
                Vector3f current_global_normal = current_global_rotation * frame_data[cur_idx].normal;                

                auto normal_angle = acos(previous_global_normal.dot(current_global_normal) / 
                (previous_global_normal.norm() * current_global_normal.norm()));
                normal_angle = normal_angle * 180/PI;
                if(normal_angle < angle_threshold){
                    std::lock_guard<std::mutex> guard(mut);
                    selected_matches[cur_idx] = prv_idx;
                }
            }
        });

        // for evaluation

        clock_t end = clock();
        double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
        if(DEBUG){
        std::cout << "Data association completed in " << elapsedSecs << " seconds. " << std::endl;
        std::cout << "find match pairs: " << matches.size() << ", after detection: " << selected_matches.size() << std::endl;
        }



        //step 2.2 ICP


        // prepare data for ICP
        // for parallel we consider caculate each point pair

        // Eigen::Vector3f s; // sourcePoint
        // Eigen::Vector3f d; // targetPoint
        // Eigen::Vector3f n;// targetNormal
        const size_t N = selected_matches.size();
        MatrixXf A = MatrixXf::Zero(6, 6);
        Eigen::VectorXf b= VectorXf::Zero(6);

        // this part can parr
        int i = 0;
        for(auto it = selected_matches.begin(); it != selected_matches.end(); ++it){
            auto source_idx = it->first;
            auto target_idx = it->second;            
            // Note that sourcePoint always in camera space
            Eigen::Vector3f s  = current_global_rotation * Vector4fToVector3f(frame_data[source_idx].position) + current_global_translation;
            Eigen::Vector3f d  = Vector4fToVector3f(model_data[target_idx].position);
            Eigen::Vector3f n  = model_data[target_idx].normal;

            // cout << "s is: " << s << endl << "d is: " << d << endl << "n is " << n << endl;

        //     Matrix<float, 1, 6> point2plane; 
        //     point2plane << n(2)*s(1)-n(1)*s(2), n(0)*s(2)-n(2)*s(0), n(1)*s(0)-n(0)*s(1), n(0), n(1), n(2);
        //     A.block<1,6>(i,0) = point2plane;
        //     // part of point2plane, copied from paper
        //     b(i) = (n(0)*d(0) + n(1)*d(1) + n(2)*d(2) - n(0)*s(0) - n(1)*s(1) - n(2)*s(2));
        //     // ICP incremental caculate
        //     ++i;
        // }
            MatrixXf  G = MatrixXf::Zero(3,6);
            G(0, 0) = 0;
            G(1, 1) = 0;
            G(2, 2) = 0;
            G(0, 1) = -s[2];
            G(0, 2) = s[1];
            G(1, 2) = -s[0];
            G(1, 0) = s[2];
            G(2, 0) = -s[1];
            G(2, 1) = s[0];
            G(0, 3) = 1;
            G(1, 4) = 1;
            G(2, 5) = 1;

            Eigen::MatrixXf tempA = MatrixXf::Zero(6, 6);
            Eigen::VectorXf tempbias = VectorXf::Zero(6);
            tempA = G.transpose() * n * n.transpose() * G;
            tempbias = G.transpose() * n * n.transpose() * (d - s);

            A = A + tempA;
            b = b + tempbias;    
        }    

        VectorXf pose_increment;
        pose_increment = A.ldlt().solve(b);

        float alpha = pose_increment(0), beta = pose_increment(1), gamma = pose_increment(2);

        Matrix3f rotation_incremental = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
                                        AngleAxisf(beta,  Vector3f::UnitY()).toRotationMatrix() *
                                        AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();    

        Vector3f translation_incremental = pose_increment.tail(3); 

        current_global_translation = rotation_incremental * current_global_translation + translation_incremental;
        current_global_rotation = rotation_incremental * current_global_rotation;
        if(DEBUG){
        cout << "incremental is: " << endl << translation_incremental << endl; 
        cout << "current rotation is :" << endl << current_global_rotation << endl;
        cout << "current translation is: " << endl << current_global_translation << endl;  
        }
    }
    
    // iteration finished

    // step 5: return new pose
    cur_pose.block<3,3>(0,0) = current_global_rotation;
    cur_pose.block<3,1>(0,3) = current_global_translation;

    if(DEBUG){
    std::cout << "Optimization iteration done " << std::endl;
    }    
    
    return true;
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
// VectorXf estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, 
//                                   const std::vector<Vector3f>& targetPoints, 
//                                   const std::vector<Vector3f>& targetNormals
//                                   ) {

    
//     const unsigned int nPoints = sourcePoints.size();
//     // Build the system
//     MatrixXf A = MatrixXf::Zero(nPoints, 6);
//     VectorXf b = VectorXf::Zero(nPoints);

//     for (unsigned i = 0; i < nPoints; i++) {
//         const auto& s = sourcePoints[i];
//         const auto& d = targetPoints[i];
//         const auto& n = targetNormals[i];

//         //Add the point-to-plane constraints to the system
//         Matrix<float, 1, 6> point2plane; 
//         point2plane << n(2)*s(1)-n(1)*s(2), n(0)*s(2)-n(2)*s(0), n(1)*s(0)-n(0)*s(1), n(0), n(1), n(2);
//         A.block<1,6>(i,0) = point2plane;

//         // part of point2plane, copied from paper
//         b(i) = (n(0)*d(0) + n(1)*d(1) + n(2)*d(2) - n(0)*s(0) - n(1)*s(1) - n(2)*s(2));

//     }    

//     VectorXf x(6);
//     x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);    

//     return x;

// }

// util for transfrom from Vector4f to Vector3f

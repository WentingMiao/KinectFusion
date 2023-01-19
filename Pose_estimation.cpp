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

    // step 1: Initialize pose 
    Eigen::Matrix4f prv_pose = cur_pose;


    // step 2: Data association
    // Note that we use current points in k project to k-1, but still find correspondence

    for(int it = 0; it < num_iteration; ++it){
        // step 2.1: data association
        clock_t begin = clock();

        const unsigned nPoints = frame_data.size();
        std::unordered_map<int, int> matches;
        std::unordered_map<int, int> selected_matches;
        int count = 0;

        for(int i = 0; i < nPoints; ++i){

            // current_vertex: in camera space of k
            // current_camera_vertex: this vertex in camera space of k-1
            
            Vector3f current_vertex = Vector4fToVector3f(frame_data[i].position);
            Vector3f current_normal = frame_data[i].normal;
            Vector3f previous_vertex = Vector4fToVector3f(model_data[i].position);
            Vector3f previous_normal = Vector4fToVector3f(model_data[i].position);
            // get vertex in previous global vertex  V_g_k-1

            // avoid redundant caculation to speed up 
            if (!isnan(current_normal[0]) && !isnan(current_normal[1]) && !isnan(current_normal[2]) &&
                !isnan(current_vertex[0]) && !isnan(current_vertex[0]) && !isnan(current_vertex[0]) &&
                current_normal[0] != MINF && current_normal[2] != MINF && current_normal[2] != MINF &&
                current_vertex[0] != MINF && current_vertex[2] != MINF && current_vertex[2] != MINF){
                
                Eigen::MatrixXf tmpTransformation = prv_pose.inverse() * cur_pose;
                Eigen::Vector3f current_camera_vertex = TransformToVertex(current_vertex, tmpTransformation);
                // transfrom to camera coordinate v_k-1

                // Eigen::Vector3f current_camera_vertex = previous_global_rotation_inverse * (current_global_vertex - previous_global_translation);            
 
                // back-project to pixel v_k-1
                Eigen::Vector2i point;
                point.x() = std::round(current_camera_vertex.x() * fX / current_camera_vertex.z() + cX);
                point.y() = std::round(current_camera_vertex.y() * fY / current_camera_vertex.z() + cY);

                // check if pixel still in image
                // && current_camera_vertex.z() >= 0
                if(point.x() >= 0 && point.y() >= 0 && point.x() < width && point.y() < height ){
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

            Vector3f current_global_vertex = TransformToVertex(Vector4fToVector3f(frame_data[cur_idx].position), cur_pose);
            Vector3f current_global_normal = TransformToNormal(frame_data[cur_idx].normal, cur_pose); 
            Vector3f previous_global_vertex = TransformToVertex(Vector4fToVector3f(model_data[prv_idx].position), prv_pose);
            Vector3f previous_global_normal = TransformToNormal(model_data[prv_idx].normal, prv_pose);

            // Vector3f current_vertex = Vector4fToVector3f(frame_data[cur_idx].position);

            // Vector3f current_global_vertex = current_global_rotation * current_vertex + current_global_translation;
            
            // Vector3f previous_global_vertex = Vector4fToVector3f(model_data[prv_idx].position);
            
            // Vector3f previous_global_normal = model_data[prv_idx].normal;

            // avoid redundant caculation to speed up 
            if (!isnan(current_global_vertex[0]) && !isnan(current_global_vertex[1]) && !isnan(current_global_vertex[2]) &&
                !isnan(previous_global_vertex[0]) && !isnan(previous_global_vertex[0]) && !isnan(previous_global_vertex[0]) &&
                current_global_vertex[0] != MINF && current_global_vertex[2] != MINF && current_global_vertex[2] != MINF &&
                previous_global_vertex[0] != MINF && previous_global_vertex[2] != MINF && previous_global_vertex[2] != MINF){

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
                    }
                }
            }
        });

        // for evaluation

        clock_t end = clock();
        double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
        if(DEBUG){
            std::cout << "Data association completed in " << elapsedSecs << " seconds. " << std::endl;
            std::cout << "find match pairs: " << matches.size() << ", after detection: " << selected_matches.size() << std::endl;
            if(!selected_matches.size()) throw std::out_of_range("No match pairs, some error exist!!");

        }


        //step 2.2 ICP
        if(DEBUG){
        cout << "initial pose is :" << endl << cur_pose << endl;
        }

        // prepare data for ICP
        // for parallel we consider caculate each point pair

        Vector3f s; // sourcePoint
        Vector3f d; // targetPoint
        Vector3f n;// targetNormal
        const size_t N = selected_matches.size();
        MatrixXf A = MatrixXf::Zero(N, 6);
        VectorXf b = VectorXf::Zero(N);

        //TODO this part can parr
        int i = 0;
        for(auto it = selected_matches.begin(); it != selected_matches.end(); ++it){
            auto source_idx = it->first;
            auto target_idx = it->second;            
            // Note that sourcePoint always in camera space
            s  = TransformToVertex(Vector4fToVector3f(frame_data[source_idx].position),cur_pose);
            d  = TransformToVertex(Vector4fToVector3f(model_data[target_idx].position),prv_pose);
            n  = TransformToNormal(model_data[target_idx].normal,prv_pose);
            A.block<1, 3>(i, 0) = s.cross(n).transpose();
            A.block<1, 3>(i, 3) = n.transpose();
            // part of point2plane, copied from paper
            b(i) = (d - s).dot(n);
            ++i;
            // ICP incremental caculate

        }

        
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

        cur_pose = pose_incremental * cur_pose;
        // current_global_translation = rotation_incremental * current_global_translation + translation_incremental;
        // current_global_rotation = rotation_incremental * current_global_rotation;
        if(DEBUG){
            cout << "iteration number: " << it << endl;
            cout << "incremental is: " << endl << pose_incremental << endl; 
            cout << "new pose is: " << endl << cur_pose << endl;
        }
    
        // iteration finish
    } 
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

Vector3f Pose::TransformToNormal(Vector3f normal, Eigen::Matrix4f Transformation)
{
    return Transformation.block(0, 0, 3, 3) * normal;

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

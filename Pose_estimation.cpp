#include "Pose_estimation.h"

#define PI acos(-1)
/**
 * Convention: 
 * we focus on Rotation R and Translation t
 * and we always process pose from frame 1 to frame 2
 * R_w1, t_w1 means parameters in world coordinate of frame 1
 * R_c2, t_c2 means parametersin camera coordinate of frame 2
 */
double total_time = 0;

bool Pose::pose_estimation(const std::vector<Vertex>& frame_data,
                     const std::vector<Vertex>& model_data,
                     const Matrix3f &Intrinsics,
                     const float distance_threshold,
                     const float angle_threshold, //with angle °
                     const int& num_iteration,
                     const unsigned int& width,
                     const unsigned int& height,
                     const unsigned int& pyramid_level,
                     const Eigen::Matrix4f& cur_pose
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
        // we store all corresponde point pair in match?
        // TODO we should directly process corresponde point to Ax-b = 0

        clock_t begin = clock();

        const unsigned nPoints = frame_data.size();
        std::vector<Match> matches(nPoints);
        int count = 0;

        for(int i = 0; i < nPoints; ++i){

            // current_vertex: in camera space of k
            // previous_vertex: this vertex in camera space of k-1
            Vector3f current_vertex;
            current_vertex.x() = frame_data[i].position.x();
            current_vertex.y() = frame_data[i].position.y();
            current_vertex.z() = frame_data[i].position.z();
            Vector3f current_normal = frame_data[i].normal;
            // std::cout << frame_data[i].position << std::endl;
            // get vertex in previous global vertex  V_g_k-1
            if (!isnan(current_normal[0]) && !isnan(current_normal[1]) && !isnan(current_normal[2]) &&
                !isnan(current_vertex[0]) && !isnan(current_vertex[0]) && !isnan(current_vertex[0]) &&
                current_normal[0] != MINF && current_normal[2] != MINF && current_normal[2] != MINF &&
                current_vertex[0] != MINF && current_vertex[2] != MINF && current_vertex[2] != MINF){
                Eigen::Vector3f current_global_vertex = current_global_rotation * current_vertex + current_global_translation;
                Eigen::Vector3f current_global_normal = current_global_rotation * current_normal;
                // transfrom to camera coordinate v_k-1
                Eigen::Vector3f previous_vertex = previous_global_rotation_inverse * (current_global_vertex - previous_global_translation);            
            // make sure normal exist
            
                
                // back-project to pixel v_k-1
                Eigen::Vector2i point;
                point.x() = std::floor(previous_vertex.x() * fX / previous_vertex.z() + cX);
                point.y() = std::floor(previous_vertex.y() * fY / previous_vertex.z() + cY);

                // check if pixel still in image
                if(point.x() >= 0 && point.y() >= 0 && point.x() < width && point.y() < height && previous_vertex.z() >= 0){
                    //cacluate v
                    
                    unsigned int previous_idx = point.y() * width + point.x();
                    // i means point in frame k
                    // idx means point in frame k-1
                    matches[i].idx = previous_idx;
                    ++count;
                    // source
                    Eigen::Vector3f previous_global_vertex;
                    previous_global_vertex.x() = model_data[previous_idx].position.x();
                    previous_global_vertex.y() = model_data[previous_idx].position.y();
                    previous_global_vertex.z() = model_data[previous_idx].position.z();
                    Eigen::Vector3f previous_global_normal = model_data[previous_idx].normal;

                    // distance check
                    const float distance = (previous_global_vertex - current_global_vertex).norm();
                    // std::cout << "distance: " << distance <<std::endl;
                    if (distance < distance_threshold){
                        //caculate norm
                        // std::cout << "entry distance" << std::endl;
                        //! normal and vertex of frame_data is in camera space
                        //! normal and vertex of model_data is in world space
                        Vector3f current_global_normal = current_global_rotation * frame_data[i].normal;
                        Vector3f previous_global_normal = model_data[i].normal;
                        // const float normal_angle = abs(current_global_normal.dot(previous_global_normal));
                        auto normal_angle = acos(previous_global_normal.dot(current_global_normal) / 
                        (previous_global_normal.norm() * current_global_normal.norm()));
                        normal_angle = normal_angle * 180/PI;
                        if(normal_angle > angle_threshold){
                            matches[i].idx = -1;
                            --count;
                        }
                    }
                }
                else{
                    matches[i].idx = -1;
                }
            } 
        }

        // for evaluation
        clock_t end = clock();
        double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Data association completed in " << elapsedSecs << " seconds. " << "with " << count << " match pairs" << std::endl;

        //step 2.2 ICP

        // prepare data for ICP

        // std::vector<Vector3f> sourcePoints;
        // std::vector<Vector3f> targetPoints;
        // std::vector<Vector3f> targetNormal;

        // // gather matched point-pair
        // for (int j = 0; j < nPoints; ++j) {
        //     const auto& match = matches[j];
        //     if (match.idx >= 0) {
        //         sourcePoints.push_back(frame_data[j].position);
        //         targetPoints.push_back(model_data[j].position);
        //         targetNormal.push_back(model_data[j].normal);
        //     }
        // }

        // // step 3: solve Ax = b -> get increment value
        // auto pose_increment = estimatePosePointToPlane(sourcePoints, targetPoints, targetNormal);

        // float alpha = pose_increment(0), beta = pose_increment(1), gamma = pose_increment(2);

        // Matrix3f rotation_incremental = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
        //                                 AngleAxisf(beta,  Vector3f::UnitY()).toRotationMatrix() *
        //                                 AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();    

        // Vector3f translation_incremental = pose_increment.tail(3);        

        // // step 4: caculate current pose after increment
        // // R_w2 = R_12 * R_w1
        // // t_w2 = R_12 * tw1 + t12
        // current_global_translation = rotation_incremental * current_global_translation + translation_incremental;
        // current_global_rotation = rotation_incremental * current_global_rotation;


        // total_time += double(clock() - end) / CLOCKS_PER_SEC;
        // std::cout << "Optimization iteration done: " << it << std::endl;
    }
    
    // iteration finished

    // step 5: return new pose
    // cur_pose.block(0, 0, 3, 3) = current_global_rotation;
    // cur_pose.block(0, 3, 3, 1) = current_global_translation;
    
    return true;
}


// VectorXd estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, 
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
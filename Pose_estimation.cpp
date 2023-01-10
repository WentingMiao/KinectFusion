#include "Pose_estimation.h"


/**
 * Convention: 
 * we focus on Rotation R and Translation t
 * and we always process pose from frame 1 to frame 2
 * R_w1, t_w1 means parameters in world coordinate of frame 1
 * R_c2, t_c2 means parametersin camera coordinate of frame 2
 */
double total_time = 0;

bool pose_estimation(const std::vector<Vertex>& frame_data,
                     const std::vector<Vertex>& model_data,
                     const Matrix3f &Intrinsics,
                     const float distance_threshold,
                     const float angle_threshold,
                     const int& num_iteration,
                     const unsigned int &width,
                     const unsigned int &height,
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
        // we store all corresponde point pair in match?
        // TODO we should directly process corresponde point to Ax-b = 0

        clock_t begin = clock();

        const unsigned nPoints = frame_data.size();
        std::vector<Match> matches(nPoints);
        int count = 0;

        for(int i = 0; i < nPoints; ++i){
            // get vertex in previous global vertex  V_g_i-1
            Eigen::Vector3f previous_global_vertex (model_data[i].position.x(), model_data[i].position.y(), model_data[i].position.z());
            // transfrom to camera coordinate v_i-1
            Eigen::Vector3f previous_vertex = previous_global_rotation_inverse * (previous_global_vertex - previous_global_translation);

            // // transform to glogbal coordinate V_k_2 = R_w2 * V_k + t_w2
            // Eigen::Vector3f current_global_vertex = current_global_rotation * current_global_vertex + current_global_translation;
            // // inverse to previous frame V_k-1 = R_w1^-1 * (V_k_1 - t_w1)
            // Eigen::Vector3f previous_vertex = previous_global_rotation_inverse *(current_global_vertex - previous_global_translation);

            // back-project to pixel v_k-1
            Eigen::Vector2i point;
            point.x() = std::floor(previous_vertex.x() * fX / previous_vertex.z() + cX);
            point.y() = std::floor(previous_vertex.y() * fY / previous_vertex.z() + cY);

            // check if pixel still in image
            if(point.x() >= 0 && point.y() >= 0 && point.x() < width && point.y() < height && previous_vertex.z() >= 0){
                //cacluate v
                Eigen::Vector3f current_vertex;
                const float current_depth = frame_data[i].position.z();
                current_vertex.x() = (point.x() - cX) / fX * current_depth;
                current_vertex.y() = (point.y() - cY) / fY * current_depth;
                current_vertex.z() = current_depth;

                Eigen::Vector3f current_global_vertex = current_global_rotation * current_vertex + current_global_translation;

                // distance check
                const float distance = (current_global_vertex - previous_global_vertex).norm();
                if (distance < distance_threshold){
                    //caculate norm
                    //! normal and vertex of frame_data is in camera space
                    //! normal and vertex of model_data is in world space
                    Vector3f current_global_normal = current_global_rotation * frame_data[i].normal;
                    Vector3f previous_global_normal = model_data[i].normal;
                    const float normal_angle = abs(current_global_normal.dot(previous_global_normal));
                    if(normal_angle < angle_threshold){
                        matches[i].idx = i;
                        ++count;
                    }
                }
            }
            else{
                matches[i].idx = -1;
            }
        } 

        // for evaluation
        clock_t end = clock();
        double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Data association completed in " << elapsedSecs << " seconds." << std::endl;

        //step 2.2 ICP

        // prepare data for ICP

        std::vector<Vector3f> sourcePoints;
        std::vector<Vector3f> targetPoints;
        std::vector<Vector3f> targetNormal;

        // gather matched point-pair
        for (int j = 0; j < nPoints; ++j) {
            const auto& match = matches[j];
            if (match.idx >= 0) {
                sourcePoints.push_back(frame_data[j].position);
                targetPoints.push_back(model_data[j].position);
                targetNormal.push_back(model_data[j].normal);
            }
        }

        // step 3: solve Ax = b -> get increment value
        auto pose_increment = estimatePosePointToPlane(sourcePoints,targetPoints,targetNormal);

        float alpha = pose_increment(0), beta = pose_increment(1), gamma = pose_increment(2);

        Matrix3f rotation_incremental = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
                                        AngleAxisf(beta,  Vector3f::UnitY()).toRotationMatrix() *
                                        AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();    

        Vector3f translation_incremental = pose_increment.tail(3);        

        // step 4: caculate current pose after increment
        // R_w2 = R_12 * R_w1
        // t_w2 = R_12 * tw1 + t12
        current_global_translation = rotation_incremental * current_global_translation + translation_incremental;
        current_global_rotation = rotation_incremental * current_global_rotation;


        total_time += double(clock() - end) / CLOCKS_PER_SEC;
        std::cout << "Optimization iteration done: " << it << std::endl;
    }
    // iteration finished

    // step 5: return new pose
    cur_pose.block(0, 0, 3, 3) = current_global_rotation;
    cur_pose.block(0, 3, 3, 1) = current_global_translation;
    
    return true;
}


VectorXd estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, 
                                  const std::vector<Vector3f>& targetPoints, 
                                  const std::vector<Vector3f>& targetNormals,
                                  ) {

    
    const unsigned int nPoints = sourcePoints.size();
    // Build the system
    MatrixXf A = MatrixXf::Zero(nPoints, 6);
    VectorXf b = VectorXf::Zero(nPoints);

    for (unsigned i = 0; i < nPoints; i++) {
        const auto& s = sourcePoints[i];
        const auto& d = targetPoints[i];
        const auto& n = targetNormals[i];

        //Add the point-to-plane constraints to the system
        Matrix<float, 1, 6> point2plane; 
        point2plane << n(2)*s(1)-n(1)*s(2), n(0)*s(2)-n(2)*s(0), n(1)*s(0)-n(0)*s(1), n(0), n(1), n(2);
        A.block<1,6>(i,0) = point2plane;

        // part of point2plane, copied from paper
        b(i) = (n(0)*d(0) + n(1)*d(1) + n(2)*d(2) - n(0)*s(0) - n(1)*s(1) - n(2)*s(2));

    }    

    VectorXf x(6);
    x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);    

    return x;

}
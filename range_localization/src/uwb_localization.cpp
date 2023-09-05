#include "range_localization/uwb_localization.hpp"


//------------------------------------------
//------------------------------------------

uavos::UWB_Node::UWB_Node(const std::string node_id):m_node_id(node_id){

}

// ---------------------------------------------
uavos::UWB_Anchor::UWB_Anchor(const std::string node_id, const Eigen::Vector3d p, const double t)
    :UWB_Node(node_id)
{
    setPosition(p.x(), p.y(), p.z());
    m_last_update_time = t;

}


// void uavos::UWB_Anchor::insertRangeInfo(const RangeInfo& msg){
//     // only insert range info that is requested by myself.

//     int requester_id = msg.nodeId;
//     int responder_id = msg.responderId;
//     double range = msg.precisionRangeM;

//     if(m_node_id==requester_id){
//         // if haven't seen the responder yet, create it
//         if(m_range_map.find(responder_id)==m_range_map.end()){
//             m_range_map.insert(std::pair<int, std::vector<double> >(responder_id, std::vector<double>()));
//             // add the range measurement into the vector
//             m_range_map.find(responder_id)->second.push_back(range);
//         } else {
//             // add the range measurement into the vector
//             m_range_map.find(responder_id)->second.push_back(range);
//         }
//     }
// }

// double uavos::UWB_Anchor::getAverageRangeToNode(const int anchor_id){
//     // check whether the anchor_id has been seen
//     std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
//     if(it==m_range_map.end()){
//         return 0;
//     } else {
//         // check the size of the vector
//         if(it->second.size()==0){
//             return 0;
//         } else {
//             return uavos::robustAverage<double>(it->second);
//         }
//     }
// }

// bool uavos::UWB_Anchor::getMedianAndVarianceToNode(const int anchor_id, double &median, double &variance){
//     std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
//     if(it==m_range_map.end()){
//         return false;
//     } else {
//         if(it->second.size()==0){
//             return false;
//         } else {
//             uavos::medianAndVariance<double>(it->second, median, variance);
//             return true;
//         }
//     }
// }



// ----------------------------------------------
uavos::UWB_Mobile::UWB_Mobile(const std::string node_id)
    :UWB_Node(node_id)
{
   
    m_kalman_sigma_a = 0.125;
    m_snr_threshold = -100;  // it is wrong when saturated!
    // general parameters

    m_pressure_is_update = false;
    m_imu_is_inited = false;
    #ifdef gt
    logRFFile.open(std::getenv("HOME") + std::string("/localization_ros2_ws/log/") + m_node_id + std::string(".csv"),std::ios::out);
    logRFFile<<  "t,buav_1,buav_2,buav_3,suav_1,suav_2,suav_3,suav_4,suav_5,suav_6,suav_7,suav_8,suav_9,suav_10,suav_11,suav_12,suav_13,suav_14,tuav_1,tuav_2,tuav_3,usv"<<std::endl;;
    #endif
    if(m_log_residual)
    {
        logResidualFile.open(DEBUG_FILE_DIR("residual.txt"),std::ios::out);
        log_predict_state.open(DEBUG_FILE_DIR("predict.txt"),std::ios::out);
        log_update_state.open(DEBUG_FILE_DIR("update1.txt"),std::ios::out);
        log_update_pressure.open(DEBUG_FILE_DIR("update2.txt"),std::ios::out);
        if (logResidualFile && log_predict_state)
            std::cout << "~~~~"<<ROOT_DIR<<" file opened" << std::endl;
        else
            std::cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << std::endl;
    }
}

void uavos::UWB_Mobile::simpleInitializeEKF(NodeParameters initParameter){
    setPosition(initParameter.position.x() ,initParameter.position.y(), initParameter.position.z());
    printf("init Position: %lf, %lf, %lf \n",initParameter.position.x() ,initParameter.position.y(), initParameter.position.z());
    setVelocity(0,0,0);
    setAccelerationBias(0,0,0);
    setStateCovariance(Eigen::MatrixXd::Identity(7,7));
    m_kalman_sigma_a = initParameter.kalman_sigma_a;
    m_z_damping_factor = initParameter.z_damping_factor;
    m_Q_scale = initParameter.Q_scale;
    m_R_scale = initParameter.R_scale;
    m_tao_acc_sqrt = initParameter.tao_acc_sqrt;
    m_tao_bias_sqrt = initParameter.tao_bias_sqrt;
    m_sigma_pressure = initParameter.sigma_pressure; 
    m_last_update_time = -1;
    m_last_predict_time = -1;
}

// void uavos::UWB_Mobile::getInnovation(){
//     bool previous_is_stablized_flag = m_is_stablized;
  
//     if(false==m_is_stablized){
//         m_innovation_threshold = m_initialization_innovation_threshold;
//         m_is_stablized = checkStablization();
//     }
//     if(true==m_is_stablized){
//         if(previous_is_stablized_flag==false){
//             m_innovation_threshold = m_normal_innovation_threshold;
//         }
//         // dynamic innovation
//         if(true==m_is_last_filter_succeed){
//             m_innovation_threshold += m_innovation_threshold_inc_on_succ;
//         } else {
//             m_innovation_threshold += m_innovation_threshold_inc_on_fail;
//         }
        
//         // upper and lower limit
//         if(m_innovation_threshold<m_normal_innovation_threshold){
//             m_innovation_threshold = m_normal_innovation_threshold;
//         } else if(m_innovation_threshold>m_initialization_innovation_threshold){
//             m_innovation_threshold = m_initialization_innovation_threshold;
//         }
//     }
    
// }


// bool uavos::UWB_Mobile::checkStablization(){
//     if(m_is_stablized==true){
//         return true;
//     }
//     std::pair<double, Eigen::Vector3d> stamp_position;
//     stamp_position.first = this->getUpdateTime();
//     stamp_position.second = this->getPosition();

//     m_init_inspector_position_array.push_back(stamp_position);

//     // get time nearby positions
//     std::vector<Eigen::Vector3d> valid_position_array;
//     for(int i = m_init_inspector_position_array.size()-1 ; i >= 0 ; --i){
//         Eigen::Vector3d position = m_init_inspector_position_array.at(i).second;
//         if(this->m_last_update_time - m_init_inspector_position_array.at(i).first < 5){
//             valid_position_array.push_back(position);
//         } else {
//             break;
//         }
//     }

//     if(valid_position_array.size()<10){
//         return false;
//     } else {
//         // get x,y,z mean
//         float x_sum=0;
//         float y_sum=0;
//         float z_sum=0;
//         for(int i=0;i<valid_position_array.size();++i){
//             Eigen::Vector3d position = valid_position_array.at(i);
//             x_sum += position.x();
//             y_sum += position.y();
//             z_sum += position.z();
//         }
//         float x_mean = x_sum / float(valid_position_array.size());
//         float y_mean = y_sum / float(valid_position_array.size());
//         float z_mean = z_sum / float(valid_position_array.size());

//         float squared_residual_error_sum_x = 0;
//         float squared_residual_error_sum_y = 0;
//         float squared_residual_error_sum_z = 0;
//         for(int i=0;i<valid_position_array.size();++i){
//             Eigen::Vector3d position = valid_position_array.at(i);
//             squared_residual_error_sum_x += std::pow(position.x()-x_mean, 2);
//             squared_residual_error_sum_y += std::pow(position.y()-y_mean, 2);
//             squared_residual_error_sum_z += std::pow(position.z()-z_mean, 2);
//         }
//         float sigma_x = std::sqrt( squared_residual_error_sum_x / float(valid_position_array.size()) );
//         float sigma_y = std::sqrt( squared_residual_error_sum_y / float(valid_position_array.size()) );
//         float sigma_z = std::sqrt( squared_residual_error_sum_z / float(valid_position_array.size()) );


//         printf("[Check Init] Sigma in the last 5 second (x, y, z): %f, %f, %f\n", sigma_x, sigma_y, sigma_z);
//         // if(sigma_x<0.5 && sigma_y<0.5 && sigma_z<0.5)
//         if(sigma_x<1.5 && sigma_y<1.5 && sigma_z<1.5)
//         {
//             return true;
//         } else {
//             return false;
//         }
//     }
// }


bool uavos::UWB_Mobile::acc_body_to_NWU(const Eigen::Quaterniond q_b_to_w, const Eigen::Vector3d acc_body, NavigationState &uav_state)
{
    static Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();

    uav_state.pose.orientation = q_b_to_w;
    Eigen::Vector3d acc_world = q_b_to_w * acc_body;
    static int _init_imu_bias = 600;
    if(_init_imu_bias > 0)
    {
        printf("\033[32m Please keep static! \033[0m\n");
        acc_bias += acc_world;
        _init_imu_bias -= 1;
        if(_init_imu_bias == 0)
        {
            acc_bias /= 600;
            m_imu_is_inited = true;
            printf("\033[31m Can begin to move! \033[0m\n");
            std::cout<<acc_bias<<std::endl;
        }
        return false;
    }
    uav_state.acceleration.linear.x() = acc_world.x() - acc_bias.x();
    uav_state.acceleration.linear.y() = acc_world.y() - acc_bias.y();
    uav_state.acceleration.linear.z() = acc_world.z() - acc_bias.z();

    // {
    //     printf("acc_free_world: %lf, %lf, %lf.\n", m_uav_state.acceleration.linear[0], m_uav_state.acceleration.linear[1] ,m_uav_state.acceleration.linear[2]);
    // } 

    return true;
}


//constant vx vy, and constant az
void uavos::UWB_Mobile::ekf_predict(const Eigen::Quaterniond imu_ori, const Eigen::Vector3d imu_acc, const double t_imu){

    if(!acc_body_to_NWU(imu_ori, imu_acc, m_uav_state))
    {
        return;
    }
    if(m_last_predict_time == -1)
    {
        m_last_predict_time = t_imu;
        return;
    }
    // ROS_INFO("acc_x: %lf, acc_y: %lf, acc_z: %lf", m_uav_state.acceleration.linear.x, m_uav_state.acceleration.linear.y, m_uav_state.acceleration.linear.z);
    Eigen::Matrix<double, 7, 1> X;
    X(0) = m_position.x();
    X(1) = m_velocity.x();
    X(2) = m_position.y();
    X(3) = m_velocity.y();
    X(4) = m_position.z();
    X(5) = m_velocity.z();
    X(6) = m_acc_bias.z();
    // logxFile<<std::fixed << std::setprecision(6) << X(0) << "," << X(3) << "," << X(6) << ",";

    double T = t_imu - ((m_last_predict_time >= m_last_update_time)?m_last_predict_time:m_last_update_time);
    if(T < 0)
    {
        printf("Error delta T: %lf. t_imu: %lf, m_last_predict_time: %lf, m_last_update_time: %lf.\n", T, t_imu, m_last_predict_time, m_last_update_time);
        return;
    }


    // printf("T: %lf.\n", T);
    
    Eigen::Matrix<double, 7, 7> F;
    F << 1,  T,  0,  0,  0,  0, 0,
         0 , 1,  0,  0,  0,  0, 0,
         0,  0,  1,  T,  0,  0, 0,
         0,  0,  0,  1,  0,  0, 0,
         0,  0,  0,  0,  1,  T, -0.5*T*T,
         0,  0,  0,  0,  0,  1, -T,
         0,  0,  0,  0,  0,  0, 1;

    Eigen::Matrix<double, 3, 1> u(0, 0, m_uav_state.acceleration.linear.z());
    Eigen::Matrix<double, 7, 3> B = Eigen::Matrix<double, 7, 3>::Zero();
    B(4,2) = 0.5 * T * T;
    B(5,2) =  T;
    // X is the predicted state vector and the predicted covariance matrix
    X = F*X + B * u;
    // log predict
    // logxFile<<std::fixed << std::setprecision(6) << X(0) << "," << X(3) << "," << X(6) << ",";
    // logxFile<<std::fixed << std::setprecision(6) << X_predict(0) << "," << X_predict(3) << "," << X_predict(6) << ",";

    Eigen::MatrixXd P = getStateCovariance();

    // Q is the acceleration model
    double tao_acc = m_tao_acc_sqrt*m_tao_acc_sqrt;
    double tao_bias = m_tao_bias_sqrt*m_tao_bias_sqrt;
    Eigen::Matrix<double, 7, 7> Q = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 3, 3>  block_Q_z;
    block_Q_z <<
        std::pow(T,3)/3.0*tao_acc+std::pow(T,5)/20.0*tao_bias,  std::pow(T,2)/2*tao_acc+std::pow(T,4)/8.0*tao_bias,  -std::pow(T,3)/6*tao_bias,
        std::pow(T,2)/2.0*tao_acc+std::pow(T,4)/8.0*tao_bias ,  T*tao_acc+std::pow(T,3)/3*tao_bias                ,  -std::pow(T,2)/2*tao_bias,
        -std::pow(T,3)/6.0*tao_bias                          ,  -std::pow(T,2)/2*tao_bias                         ,  T*tao_bias               ;
    
    double tao_sigma_a = m_kalman_sigma_a*m_kalman_sigma_a;
    Eigen::Matrix<double, 2, 2>  block_Q_xy;
    block_Q_xy << std::pow(T,4)/3.0*tao_sigma_a,  std::pow(T,3)/2.0*tao_sigma_a,  
         std::pow(T,3)/2.0*tao_sigma_a,  std::pow(T,2)*tao_sigma_a;
    
    Q.block(0,0,2,2) = block_Q_xy;
    Q.block(2,2,2,2) = block_Q_xy;
    Q.block(4,4,3,3) = block_Q_z;
    Q = m_Q_scale * Q;


    // M is the predicted covariance matrix
    Eigen::Matrix<double, 7, 7> M = F*P*F.transpose() + Q;

    if(m_log_residual)
    {
        Eigen::Vector3d acc_world = imu_ori * imu_acc;
        log_predict_state << std::setprecision(15) << std::setw(20) << t_imu // [0]
        <<" " << X.transpose().matrix()  // [1-7]
        << " " << acc_world.transpose()  // [8-10]
        << std::endl;;
    }
    // printf("----------------------predict %lf---------------------------\n", t_imu);
    m_last_predict_time = t_imu;
    setPosition(X(0),X(2),X(4));
    setVelocity(X(1),X(3),X(5));
    setAccelerationBias(0,0,X(6));
    setStateCovariance(M);
}
// constant vx vy, and constant az
// bool uavos::UWB_Mobile::ekf_update_loosely(const RangeInfoVec &vec_range_info)
// {
//     const int RSIZE = vec_range_info.RangeInfos.size();

//     if(RSIZE < 4)
//     {
//         return false;
//     }
//     if(vec_range_info.stamp < m_last_predict_time)
//     {
//         printf("range update time %lf smaller than m_last_predict_time: %lf\n",vec_range_info.stamp, m_last_predict_time);
//         return false;
//     }
//     if( vec_range_info.stamp < m_last_update_time)
//     {
//         printf("range update time %lf smaller than m_last_update_time: %lf\n",vec_range_info.stamp, m_last_update_time);
//         return false;
//     }
    
//     Eigen::MatrixXd M = getStateCovariance();

//     Eigen::MatrixXd G = Eigen::MatrixXd::Zero(RSIZE, 3);
//     Eigen::Vector3d P = m_position;
//     Eigen::MatrixXd b = Eigen::MatrixXd::Zero(RSIZE, 1);
//     for(int itr =0; itr < 3; itr ++)
//     {
//         for(int i = 0; i < RSIZE; i++){
//             std::string anchor_id = vec_range_info.RangeInfos[i].responderId;
//             Eigen::Vector3d anchor_position = m_anchor_map.find(anchor_id)->second->getPosition();
//             b(i) =vec_range_info.RangeInfos[i].precisionRangeM - (P - anchor_position).norm();
//             // H is the linearized measurement matrix
//             G.block(i,0,1,3) = ((P - anchor_position) / (P - anchor_position).norm()).transpose();

//         }
//         // std::cout << "G" << G.matrix() <<std::endl;
//         // std::cout << "b" << b.matrix() <<std::endl;
//         Eigen::Vector3d dx = (G.transpose()*G).inverse()*(G.transpose())*b;
//         P = P + dx;
//         // std::cout << "P" << P.matrix() <<std::endl;
//     }
//     Eigen::Matrix3d Q_R = (G.transpose()*G).inverse();


//     Eigen::Matrix<double, 7, 1> X;
//     X(0) = m_position.x();
//     X(1) = m_velocity.x();
//     X(2) = m_position.y();
//     X(3) = m_velocity.y();
//     X(4) = m_position.z();
//     X(5) = m_velocity.z();
//     X(6) = m_acc_bias.z();
//     Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 7);
//     H(0,0) = 1;
//     H(1,2) = 1;
//     H(2,4) = 1;

//     Eigen::Matrix3d R ;
//     R << Q_R(0,0)*m_R_scale, 0, 0,
//         0, Q_R(1,1)*m_R_scale, 0,
//         0, 0,  Q_R(2,2)*m_z_damping_factor*m_R_scale;
//         // std::cout << "R" << R.matrix() <<std::endl;
//     // ROS_INFO("--------------------------------");
//     // K is the Kalman Gain
//     Eigen::MatrixXd K = M*H.transpose() * ((H*M*H.transpose()) + R ).inverse();
//     // Update P for the a posteriori covariance matrix
//     M = ( Eigen::MatrixXd::Identity(7,7) - K*H ) * M;
//     // Return the measurement innovation
//     // Update the state
//     X = X + K * (P - m_position);

//     Eigen::MatrixXd dz = K * (P - m_position);
//     // if (std::abs(dz(4)) > 1)
//     // {
//     //     printf("*********************Error occurs while range updata****************************\n");
//     //     return false;
//     // }

//     Eigen::MatrixXd e_meas = (P - m_position);
    
    
//     if(m_log_residual)
//     {
//         Eigen::Matrix<double, 3, 1> e_meas_out = Eigen::Matrix<double, 3, 1>::Zero();
//         e_meas_out.segment(0,3) = e_meas;
//         Eigen::Matrix<double, 3, 1> z_meas_out = Eigen::Matrix<double, 3, 1>::Zero();
//         z_meas_out.segment(0,3) = P;
//         Eigen::Matrix<double, 3, 1> z_pred_out = Eigen::Matrix<double, 3, 1>::Zero();
//         z_pred_out.segment(0,3) = m_position;
//         log_update_state << std::setprecision(15) << std::setw(20) << vec_range_info.stamp << " " << X.transpose().matrix() <<" "<< z_meas_out.transpose().matrix() << " " << z_pred_out.transpose().matrix() <<" "<< e_meas_out.transpose().matrix() << " " << R.diagonal().transpose() << std::endl;
//     }
//     printf("----------------------update range %lf---------------------------\n", vec_range_info.stamp);
//     m_last_update_time = vec_range_info.stamp;
//     setPosition(X(0),X(2),X(4));
//     setVelocity(X(1),X(3),X(5));
//     setAccelerationBias(0,0,X(6));
//     setStateCovariance(M);
//     return true;
// }



Eigen::Vector3d uavos::UWB_Mobile::get_LQ_result(const RangeInfoVec &vec_range_info)
{
    const int RSIZE = vec_range_info.RangeInfos.size();
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(RSIZE, 3);
    Eigen::Vector3d P = m_position;
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(RSIZE, 1);
    for(int itr =0; itr < 4; itr ++)
    {
        for(int i = 0; i < RSIZE; i++){
            std::string anchor_id = vec_range_info.RangeInfos[i].responderId;
            Eigen::Vector3d anchor_position = m_anchor_map.find(anchor_id)->second->getPosition();
            b(i) =vec_range_info.RangeInfos[i].precisionRangeM - (P - anchor_position).norm();
            // H is the linearized measurement matrix
            G.block(i,0,1,3) = ((P - anchor_position) / (P - anchor_position).norm()).transpose();
        }
        // std::cout << "G" << G.matrix() <<std::endl;
        // std::cout << "b" << b.matrix() <<std::endl;
        Eigen::Vector3d dx = (G.transpose()*G).inverse()*(G.transpose())*b;
        P = P + dx;
        // std::cout << "P" << P.matrix() <<std::endl;
    }
    Eigen::Matrix3d Q_R = (G.transpose()*G).inverse();
    m_DOP = Q_R;
    return P;
}
Eigen::Matrix<double, 4, 3> uavos::UWB_Mobile::get_anchors_position()
{
    Eigen::Matrix<double, 4, 3> anchors_position;
    int i = 0;
    for(auto itr = m_anchor_map.cbegin(); itr != m_anchor_map.cend(); itr++)
    {
        anchors_position.block(i,0,1,3) = itr->second->getPosition().transpose();
        i ++;
    }
    return anchors_position;
    
}
bool isPointInBox(Eigen::Matrix<double, 4, 3> &box, Eigen::Vector3d point)
{
    auto max = box.colwise().maxCoeff();
    auto min = box.colwise().minCoeff();

    if(point.x() < max(0) && point.x() > min(0) && point.y() < max(1) && point.y() > min(1))
    {
        return true;
    }
    return false;

}
bool uavos::UWB_Mobile::ekf_update_tightly(const RangeInfoVec &vec_range_info)
{


    const int RSIZE = vec_range_info.RangeInfos.size();

    if(RSIZE < 4)
    {
        return false;
    }
    static Eigen::Matrix<double, 4, 3> anchor_positions = get_anchors_position();

    if(isPointInBox(anchor_positions, m_position) && m_position.z() < 0.75)
    {
        m_switch_pressure_on = false;
    }
    else if(isPointInBox(anchor_positions, m_position) && m_position.z() > 0.8)
    {
        m_switch_pressure_on = true;
    }
    

    Eigen::Matrix<double, 7, 1> X;
    X(0) = m_position.x();
    X(1) = m_velocity.x();
    X(2) = m_position.y();
    X(3) = m_velocity.y();
    X(4) = m_position.z();
    X(5) = m_velocity.z();
    X(6) = m_acc_bias.z();    
    Eigen::MatrixXd P;
    Eigen::MatrixXd M = getStateCovariance();
    // std::cout << M.matrix() << std::endl;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(RSIZE, 7);
    Eigen::MatrixXd z_pred = Eigen::MatrixXd::Zero(RSIZE, 1);
    Eigen::MatrixXd z_meas = Eigen::MatrixXd::Zero(RSIZE, 1);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(RSIZE, RSIZE);

    
    for(int i = 0; i < RSIZE; i++){
        std::string anchor_id = vec_range_info.RangeInfos[i].responderId;
        double sigma_r = vec_range_info.RangeInfos[i].precisionRangeErrEst;
        double sigma_a = m_kalman_sigma_a;
        z_meas(i,0) = vec_range_info.RangeInfos[i].precisionRangeM;

        Eigen::Vector3d XYZ0 = m_anchor_map.find(anchor_id)->second->getPosition();
        z_pred(i,0) = std::sqrt( std::pow(X(0) - XYZ0(0), 2) +
                                std::pow(X(2) - XYZ0(1), 2) +
                                std::pow(X(4) - XYZ0(2), 2) ) + 1e-9;
        // H is the linearized measurement matrix
        H(i,0) = (X(0) - XYZ0(0))/z_pred(i,0);
        H(i,2) = (X(2) - XYZ0(1))/z_pred(i,0);
        H(i,4) = (X(4) - XYZ0(2))/z_pred(i,0);
        R(i,i) = std::pow(sigma_r,2) * m_R_scale;
    }

    // ROS_INFO("--------------------------------");
    // K is the Kalman Gain
    Eigen::MatrixXd K = M*H.transpose() * ((H*M*H.transpose()) + R ).inverse();
    // Update P for the a posteriori covariance matrix
    P = ( Eigen::MatrixXd::Identity(7,7) - K*H ) * M;
    // Return the measurement innovation
    double innovation = (z_meas - z_pred).norm();
    // Update the state
    X = X + K * (z_meas - z_pred);

    Eigen::MatrixXd dx = K * (z_meas - z_pred);
    // if (std::abs(dx(4)) > 1)
    // {
    //     printf("*********************Error occurs while range updata****************************\n");
    //     return false;
    // }

    Eigen::MatrixXd e_meas = z_meas - z_pred;
    
    
    if(m_log_residual)
    {
        Eigen::Vector3d LQ_P = get_LQ_result(vec_range_info);
        Eigen::Matrix<double, 4, 1> e_meas_out = Eigen::Matrix<double, 4, 1>::Zero();
        e_meas_out.segment(0,RSIZE) = e_meas;
        Eigen::Matrix<double, 4, 1> z_meas_out = Eigen::Matrix<double, 4, 1>::Zero();
        z_meas_out.segment(0,RSIZE) = z_meas;
        Eigen::Matrix<double, 4, 1> z_pred_out = Eigen::Matrix<double, 4, 1>::Zero();
        z_pred_out.segment(0,RSIZE) = z_pred;

        log_update_state << std::setprecision(15) << std::setw(20) << vec_range_info.stamp  //[0] t
        <<" " << X.transpose().matrix() // [1-7] X
        <<" "<< z_meas_out.transpose().matrix()  // [8-11] \check{z}
        <<" " << z_pred_out.transpose().matrix() // [12-15] \hat{z}
        <<" "<< e_meas_out.transpose().matrix()  // [16-19] \e_{z}
        <<" " << LQ_P.transpose().matrix() // [20-22] LQ_P
        <<" " << m_DOP.diagonal().transpose().matrix() // [23-25] LQ_P
        << std::endl;

    }
    // printf("----------------------updata tightly couple range %lf---------------------------\n", vec_range_info.stamp);
    m_last_update_time = vec_range_info.stamp;
    if (X(4) < 0)
    {
        X(4) = - X(4);
    }
    setPosition(X(0),X(2),X(4));
    setVelocity(X(1),X(3),X(5));
    setAccelerationBias(0,0,X(6));
    setStateCovariance(P);
    
    return true;
}


bool uavos::UWB_Mobile::ekf_update_pressure(const double &height, const double &t_pres)
{

    Eigen::Matrix<double, 7, 1> X;
    X(0) = m_position.x();
    X(1) = m_velocity.x();
    X(2) = m_position.y();
    X(3) = m_velocity.y();
    X(4) = m_position.z();
    X(5) = m_velocity.z();
    X(6) = m_acc_bias.z();
    
    Eigen::MatrixXd P;
    Eigen::MatrixXd M = getStateCovariance();

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1,7);
    Eigen::MatrixXd z_pred = Eigen::MatrixXd::Zero(1,1);
    Eigen::MatrixXd z_meas = Eigen::MatrixXd::Zero(1,1);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1,1);

    z_pred(0, 0) =  X(4);
    z_meas(0, 0) = height;
    // R(0, 0) = std::pow(m_sigma_pressure,2); 
    // double mormalized_DOP = std::sqrt(m_DOP(2,2))/2;
    // R(0, 0) = m_sigma_pressure/std::sqrt(m_DOP(2,2));
    // if(mormalized_DOP < 1.5)
    // {
    //     R(0, 0) = m_sigma_pressure*100;
    // }
    // else
    // {
    //     R(0, 0) = m_sigma_pressure;
    // }
    R(0, 0) = m_sigma_pressure;
    H(0, 5) = 1;

    // K is the Kalman Gain
    Eigen::MatrixXd K = M*H.transpose() * ((H*M*H.transpose()) + R ).inverse();

    // Update P for the a posteriori covariance matrix
    P = ( Eigen::MatrixXd::Identity(7,7) - K*H ) * M;

    // Return the measurement innovation
    Eigen::MatrixXd e_meas = K * (z_meas - z_pred);
    X = X + K * (z_meas - z_pred);
    // std::cout<< X(6) << std::endl; 

    if(m_log_residual)
    {
        log_update_pressure << std::setprecision(15) << std::setw(20) << t_pres  // [0]
        <<" "<< X.transpose().matrix() // [1-7]
        <<" "<< z_meas.transpose().matrix() // [8]
        <<" "<< z_pred.transpose().matrix() // [9]
        <<" "<< e_meas.transpose().matrix() // [10-16]
        <<" "<< R(0, 0) << ' ' << m_DOP(2,2) << std::endl; // [17-18]
    }

    // if((z_meas - z_pred).norm() > 0.5 )
    // {
    //     printf("\033[31m Great Gap Occurs between Pressure height and predict height! \033[0m\n");
    //     return false;
    // }
    // printf("----------------------update pressure %lf---------------------------\n", t_pres);
    m_last_update_time = t_pres;
    setPosition(X(0),X(2),X(4));
    setVelocity(X(1),X(3),X(5));
    setAccelerationBias(0,0,X(6));
    setStateCovariance(P);
    // last_pressure_height = height;
    return true;
}

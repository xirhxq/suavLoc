#include "range_localization/uwb_interface.h"
#include "enu2lla.hpp"

uavos::UWB_Localization::UWB_Localization(ros::NodeHandle &nh):m_nh(nh)
{
    std::cout<<"Begin to load paramenters"<<std::endl;
    // load param
    m_nh.param<std::string>("vehicle_name", m_vehicle_name,"uav");
    m_nh.param<std::string>("uwb_sensor_type", m_uwb_sensor_type,"nooploop");
    std::cout << "vehicle_name: " << m_vehicle_name << std::endl;
    m_nh.param<int>("slam_fps", m_slam_fps, 30);
    std::cout << "slam_fps: " << m_slam_fps << std::endl;
    m_nh.param<bool>("use_pressure_height", m_use_pressure_height,false);
    m_nh.param<bool>("is_initialize_with_ceres", m_is_initialize_with_ceres,false);
    m_nh.param<bool>("use_laser_height", m_use_laser_height,false);
    m_nh.param<bool>("use_imu", m_use_imu,false);
    m_nh.param<int>("callback_len", m_callback_len,1);
    
    // create UWB_Mobile
    
    m_p_mobile = boost::shared_ptr<uavos::UWB_Mobile>(new uavos::UWB_Mobile(m_vehicle_name));
    m_nh.param<std::string>("map_frame", m_frame_id, "world");

    m_nh.param<std::vector<std::string>>("anchor_list", m_anchor_list, std::vector<std::string>());
    // get anchor position, build anchor_map
    for(size_t i=0;i<m_anchor_list.size();++i){
        std::string anchor_id = m_anchor_list.at(i);
        std::vector<double> position;
        m_nh.param<std::vector<double>>(anchor_id, position, std::vector<double>());
        if(position.size()==3){
            Eigen::Vector3d pa(position.at(0), position.at(1), position.at(2));
            boost::shared_ptr<uavos::UWB_Anchor> p_anchor(new uavos::UWB_Anchor(anchor_id , pa, ros::Time::now().toSec()));
            m_p_mobile->m_anchor_map.insert(std::pair<std::string, boost::shared_ptr<uavos::UWB_Anchor> >(anchor_id, p_anchor) );
        }
        double temp_anchor_bias;
        m_nh.param<double>(anchor_id + "_bias", temp_anchor_bias, 0.0);
        m_p_mobile->m_anchor_bias.insert(std::pair<std::string, double>(anchor_id, temp_anchor_bias) );
    }

    NodeParameters initParameter;
    m_nh.param<double>("z_damping_factor", initParameter.z_damping_factor,0.1);
    m_nh.param<double>("Q_scale", initParameter.Q_scale,0.1);
    m_nh.param<double>("R_scale", initParameter.R_scale,0.1);
    m_nh.param<double>("tao_acc_sqrt", initParameter.tao_acc_sqrt,0.1);
    m_nh.param<double>("tao_bias_sqrt", initParameter.tao_bias_sqrt,0.1);
    m_nh.param<double>("sigma_pressure", initParameter.sigma_pressure,0.1);
    m_nh.param<double>("kalman_sigma_a", initParameter.kalman_sigma_a,0.1);
    std::vector<double> vec_position;

    m_nh.param<std::vector<double>>("init_position", vec_position, std::vector<double>());
    initParameter.position = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vec_position.data(), 3, 1);

    std::vector<double> vec_gps;
    m_nh.param<std::vector<double>>("init_gps_point", vec_gps, std::vector<double>());
    m_gps_init = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vec_gps.data(), 3, 1);

    // initialize by zero or Ceres
    if(true==m_is_initialize_with_ceres){
        // uavos::UWB_Loc_Init initializer(m_p_mobile);
        // initializer.initializeMobileByCeres();
    } else {
        m_p_mobile->simpleInitializeEKF(initParameter);
    }

    m_nh.param<std::string>("imu_topic", m_imu_sub_topic,"/"+m_vehicle_name+"/imu/data");
    printf("imu_topic: %s.\n",m_imu_sub_topic.c_str());
    m_nh.param<std::string>("pressure_topic", m_pressure_sub_topic,"/"+m_vehicle_name+"/pressure");
    m_nh.param<std::string>("motor_speed_topic", m_motor_speed_sub_topic,"/"+m_vehicle_name+"/motor_speed");
    m_nh.param<std::string>("height_topic", m_height_sub_topic,"/"+m_vehicle_name+"/pressure");
    m_nh.param<std::string>("range_topic", m_range_sub_topic,"/"+m_vehicle_name+"/uwb/data");

    if(m_use_imu)
        m_imu_sub = m_nh.subscribe(m_imu_sub_topic, m_callback_len, &uavos::UWB_Localization::imuCallback,this);
    else


    
    if(m_use_pressure_height)
    {
        m_pressure_sub = m_nh.subscribe(m_pressure_sub_topic, m_callback_len, &uavos::UWB_Localization::pressureCallback,this);
        m_motor_speed_sub = m_nh.subscribe(m_motor_speed_sub_topic, m_callback_len, &uavos::UWB_Localization::motorSpeedCallback,this);
    }
        
    if(m_use_laser_height)
    {
        // m_pressure_sub = m_nh.subscribe(m_height_sub_topic, m_callback_len, &uavos::UWB_Localization::heightCallback,this);
        m_pressure_sub = m_nh.subscribe(m_height_sub_topic, m_callback_len, &uavos::UWB_Localization::SkpHeightCallback,this);
    }
        

    if(m_uwb_sensor_type == "nooploop")
        m_range_sub = m_nh.subscribe(m_range_sub_topic, m_callback_len, &uavos::UWB_Localization::rangeOfRICallback2,this);
    else
        m_range_sub = m_nh.subscribe(m_range_sub_topic, m_callback_len, &uavos::UWB_Localization::rangeOfRICallback1,this);

    // pub pose
    pose_predict_pub = m_nh.advertise<nav_msgs::Odometry>("/" + m_vehicle_name + "/predict/odom", 10);
    pose_filter_pub = m_nh.advertise<nav_msgs::Odometry>("/" + m_vehicle_name + "/filter/odom", 10);

    pubPath = nh.advertise<nav_msgs::Path>("/" + m_vehicle_name + "/filter/path", 1);
}

void uavos::UWB_Localization::motorSpeedCallback(const std_msgs::Float32MultiArray & msg)
{   
    m_motor_speed = msg.data[0]; 
    if(m_motor_speed < 200)
        m_motor_called = false;
    else
        m_motor_called = true;

}
void uavos::UWB_Localization::pressureCallback(const std_msgs::Float32 & msg)
{
    static bool Initialized_pressure_bias = false;
    // if(!m_p_mobile->m_range_called && !m_p_mobile->m_imu_called && !m_motor_called)
    if(!m_p_mobile->m_range_called && !m_p_mobile->m_imu_called)
    {
        Initialized_pressure_bias = false;
        return;
    }
    
    double t = ros::Time::now().toSec();
    double height = msg.data;

    // initialization, check whether the height is stable
    static std::deque<double> vec_delta_pressure_uwb;
    static double ave_pressure_uwb;
    
    if(!Initialized_pressure_bias && m_p_mobile->getPosition().z() < 0.8)
    {
        return;
    }
    if(!Initialized_pressure_bias)
    {
        // double delta_height = m_p_mobile->getPosition().z() - height;
        // vec_delta_pressure_uwb.push_back( delta_height );
        // if(!check_height_stable(vec_delta_pressure_uwb, ave_pressure_uwb))
        // {
        //     printf("Height Sensor Not Initialized\n");
        //     return;
        // }
        ave_pressure_uwb = m_p_mobile->getPosition().z() - height;
        Initialized_pressure_bias = true;
        printf("\033[32m Height Sensor Initialized \033[0m \n");
    }
    double align_pressure_height = ave_pressure_uwb + height;
    if(m_p_mobile->m_switch_pressure_on)
    {
        printf("\033[32m switch pressure on \033[0m \n");
        m_p_mobile->ekf_update_pressure(align_pressure_height, t);
    }
    else
    {
        printf("\033[31m switch pressure off \033[0m \n");
        Initialized_pressure_bias = false;
    }
        
}

void uavos::UWB_Localization::heightCallback(const std_msgs::Float32 & msg)
{
    // static std::deque<double> vec_height;
    if(!m_p_mobile->m_range_called && !m_p_mobile->m_imu_called)
    {
        return;
    }
    double t = ros::Time::now().toSec();
    double height = msg.data;

    static std::deque<double> vec_height;
    
    // initialization, check whether the height is stable
    static double ave_height_uwb;
    static bool Initialized_bias = false;
    if(!Initialized_bias && !check_height_stable(m_p_mobile->getPosition().z(), ave_height_uwb))
    {
        vec_height.push_back( height );
        printf("Height Sensor Not Initialized\n");
        return;
    }
    Initialized_bias = true;
    static double delta_uwb_pressure = statistics_average(vec_height) - ave_height_uwb;

    double pressure_height = height - delta_uwb_pressure;
    
    static double last_uwb_height = m_p_mobile->getPosition().z();
    static double last_pressure_height = pressure_height;
    static bool first_pressure = true;
    if(!first_pressure)
    {
        double delta_uwb_height = abs(m_p_mobile->getPosition().z() - last_uwb_height);
        double delta_pressure_height = abs(pressure_height - last_pressure_height);
        if(delta_pressure_height - delta_uwb_height > 0.5)
        {
            printf("\033[31m Great Gap Occurs in Pressure height! \033[0m\n");
            return;
        }
        first_pressure = false;
    }

    if(last_uwb_height > 8.5)
    {
        return;
    }
    delta_uwb_pressure += 0.02 * constrain(pressure_height - m_p_mobile->getPosition().z(), -0.01, 0.01);
    printf("delta_uwb_pressure %lf\n",delta_uwb_pressure);


    m_p_mobile->ekf_update_pressure(pressure_height, t);    
}
void uavos::UWB_Localization::SkpHeightCallback(const geometry_msgs::Vector3Stamped & msg)
{
    // static std::deque<double> vec_height;
    if(!m_p_mobile->m_range_called)
    {
        return;
    }
    double t = msg.header.stamp.toSec();
    double height = msg.vector.x;

    static std::deque<double> vec_height;
    
    // initialization, check whether the height is stable
    static double ave_height_uwb;
    static bool Initialized_bias = false;
    if(!Initialized_bias && !check_height_stable(m_p_mobile->getPosition().z(), ave_height_uwb))
    {
        vec_height.push_back( height );
        printf("Height Sensor Not Initialized\n");
        return;
    }
    Initialized_bias = true;
    static double delta_uwb_pressure = statistics_average(vec_height) - ave_height_uwb;

    double pressure_height = height - delta_uwb_pressure;
    
    static double last_uwb_height = m_p_mobile->getPosition().z();
    static double last_pressure_height = pressure_height;
    static bool first_pressure = true;
    if(!first_pressure)
    {
        double delta_uwb_height = abs(m_p_mobile->getPosition().z() - last_uwb_height);
        double delta_pressure_height = abs(pressure_height - last_pressure_height);
        if(delta_pressure_height - delta_uwb_height > 0.5)
        {
            printf("\033[31m Great Gap Occurs in Pressure height! \033[0m\n");
            return;
        }
        first_pressure = false;
    }

    if(last_uwb_height > 8.5)
    {
        return;
    }
    delta_uwb_pressure += 0.02 * constrain(pressure_height - m_p_mobile->getPosition().z(), -0.01, 0.01);
    printf("delta_uwb_pressure %lf\n",delta_uwb_pressure);


    m_p_mobile->ekf_update_pressure(pressure_height, t);    
}

void uavos::UWB_Localization::imuCallback(const sensor_msgs::Imu & msg)
{
    m_p_mobile->m_imu_called = true;
    if(!m_p_mobile->m_range_called)
    {
        return;
    }

    double t = msg.header.stamp.toSec();
    // std::cout<< "IMU stamp: "<< msg.header.stamp << std::endl;
    // std::cout<< "ros stamp: "<< ros::Time::now() << std::endl;
    Eigen::Quaterniond imu_q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z); 
    Eigen::Vector3d imu_acc(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

    static Eigen::Quaterniond imu_q0(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z); 
        m_p_mobile->ekf_predict(imu_q, imu_acc, t);
    static long unsigned int downsize = 0;
    if(downsize %4 == 0)
    {
        printLocalizationCallback(t);
    }
    downsize ++;

}

void uavos::UWB_Localization::rangeOfRICallback1(const rtls_uwb_sensor::uwbs & msg)
{

    if(m_use_imu && !m_p_mobile->m_imu_called)
    {
        return;
    }
    m_p_mobile->m_range_called = true;
    RangeInfoVec range_vec;
    range_vec.stamp = msg.header.stamp.toSec();
    for(int i = 0; i < msg.uwb_vec.size(); i++){
        
        RangeInfo rangeinfo;
        rangeinfo.responderId = msg.uwb_vec[i].responderId;
        double range_bias_temp = m_p_mobile->m_anchor_bias.find(rangeinfo.responderId)->second;
        rangeinfo.precisionRangeM = msg.uwb_vec[i].precisionRangeM - range_bias_temp;
        rangeinfo.precisionRangeErrEst = msg.uwb_vec[i].precisionRangeErrEst;
        range_vec.RangeInfos.push_back(rangeinfo);
    }
    if(range_vec.RangeInfos.size() == 0)
    {
        printf("\033[31m uwb data size error. \033[0m");
    }
    if(m_p_mobile->ekf_update_tightly(range_vec))
    {

        double lla[3] = {0};
        double lla0[3] = {m_gps_init[0], m_gps_init[1], m_gps_init[2]};
        double xyz[3] = {m_p_mobile->getPosition().x(), m_p_mobile->getPosition().y(),m_p_mobile->getPosition().z()};
        enu2lla::enuToLlh(lla0, xyz, lla);

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time().fromSec(range_vec.stamp);
        odom.header.frame_id = m_frame_id;
        odom.pose.pose.position.x = xyz[0];
        odom.pose.pose.position.y = xyz[1];
        odom.pose.pose.position.z = xyz[2];
        odom.pose.pose.orientation.x = m_p_mobile->getOritation().x();
        odom.pose.pose.orientation.y = m_p_mobile->getOritation().y();
        odom.pose.pose.orientation.z = m_p_mobile->getOritation().z();
        odom.pose.pose.orientation.w = m_p_mobile->getOritation().w();
        odom.twist.twist.linear.x = m_p_mobile->getVelocity().x();
        odom.twist.twist.linear.y = m_p_mobile->getVelocity().y();
        odom.twist.twist.linear.z = m_p_mobile->getVelocity().z();
        odom.twist.twist.angular.x = lla[0];
        odom.twist.twist.angular.y = lla[1];
        odom.twist.twist.angular.z = lla[2];

        pose_filter_pub.publish(odom);

        // publish path
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;
        globalPath.poses.push_back(pose_stamped);
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header = odom.header;
            pubPath.publish(globalPath);
        }
    } else {
    }
}

void uavos::UWB_Localization::rangeOfRICallback2(const nlink_parser::LinktrackNodeframe3 & msg)
{
     if(m_use_imu && !m_p_mobile->m_imu_called)
    {
        return;
    }
    m_p_mobile->m_range_called = true;

    RangeInfoVec range_vec;
    range_vec.stamp = ros::Time::now().toSec();
    // std::cout<< "uwb stamp: "<<   range_vec.stamp << std::endl;
    // range_vec.stamp = m_p_mobile->m_last_predict_time;
    for(int i = 0; i < msg.nodes.size(); i++){
        
        RangeInfo rangeinfo;
        rangeinfo.responderId = "anchor_" + std::to_string(msg.nodes[i].id);
        // std::cout<< "msg: "<< i << "  : " << rangeinfo.responderId << std::endl;
        // double range_bias_temp = m_p_mobile->m_anchor_bias.find(rangeinfo.responderId)->second;
        rangeinfo.precisionRangeM = msg.nodes[i].dis;
        rangeinfo.precisionRangeErrEst = 0.1;
        if(msg.nodes[i].id < 4)
            range_vec.RangeInfos.push_back(rangeinfo);
    }
    // if(range_vec.RangeInfos.size() < m_anchor_list.size())
    // {
    //     printf("\033[31m T: %lf wrong uwb data size. %d/%d node connected! \033[0m\n",ros::Time::now().toSec(), range_vec.RangeInfos.size(),m_anchor_list.size());
    //     for(int i = 0; i < msg.nodes.size(); i++)
    //     {
    //         printf("%d ",msg.nodes[i].id);
    //     }
    //     printf("\n");
    // }
    // else
    // {
    //     printf("T: %lf \033[31mright uwb data size. %d/%d node connected! \033[0m\n",ros::Time::now().toSec(), range_vec.RangeInfos.size(),m_anchor_list.size());
    // }


    if(m_p_mobile->ekf_update_tightly(range_vec))
    // if(m_p_mobile->kalmanFilter3DUpdate(range_vec))
    {
        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time().fromSec(range_vec.stamp);
        odom.header.frame_id = m_frame_id;
        odom.pose.pose.position.x = m_p_mobile->getPosition().x();
        odom.pose.pose.position.y = m_p_mobile->getPosition().y();
        odom.pose.pose.position.z = m_p_mobile->getPosition().z();
        odom.pose.pose.orientation.x = m_p_mobile->getOritation().x();
        odom.pose.pose.orientation.y = m_p_mobile->getOritation().y();
        odom.pose.pose.orientation.z = m_p_mobile->getOritation().z();
        odom.pose.pose.orientation.w = m_p_mobile->getOritation().w();
        odom.twist.twist.linear.x = m_p_mobile->getVelocity().x();
        odom.twist.twist.linear.y = m_p_mobile->getVelocity().y();
        odom.twist.twist.linear.z = m_p_mobile->getVelocity().z();
        pose_filter_pub.publish(odom);

        // publish path
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;
        globalPath.poses.push_back(pose_stamped);
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header = odom.header;
            pubPath.publish(globalPath);
        }
    } else {
    }
}

void uavos::UWB_Localization::printLocalizationCallback(double stamp)
{

    // ROS_INFO("print localization");
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time().fromSec(stamp);
    odom.header.frame_id = m_frame_id;
    
    odom.pose.pose.position.x = m_p_mobile->getPosition().x();
    odom.pose.pose.position.y = m_p_mobile->getPosition().y();
    odom.pose.pose.position.z = m_p_mobile->getPosition().z();
    odom.pose.pose.orientation.x = m_p_mobile->getOritation().x();
    odom.pose.pose.orientation.y = m_p_mobile->getOritation().y();
    odom.pose.pose.orientation.z = m_p_mobile->getOritation().z();
    odom.pose.pose.orientation.w = m_p_mobile->getOritation().w();
    odom.twist.twist.linear.x = m_p_mobile->getVelocity().x();
    odom.twist.twist.linear.y = m_p_mobile->getVelocity().y();
    odom.twist.twist.linear.z = m_p_mobile->getVelocity().z();
    odom.twist.twist.angular.x = m_p_mobile->getAccelaration().x(); //acc 
    odom.twist.twist.angular.y = m_p_mobile->getAccelaration().y();
    odom.twist.twist.angular.z = m_p_mobile->getAccelaration().z();
    pose_predict_pub.publish(odom);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_node");
    ros::NodeHandle nh("~");  
    uavos::UWB_Localization RN(nh);

    
    ROS_INFO("\033[1;32m---->uwb localization started.\033[0m");

    // ros::Rate rate(100);//10HZ
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    // }
    
    ros::MultiThreadedSpinner spinner(6);
    spinner.spin();

    
    
    return 0;
}


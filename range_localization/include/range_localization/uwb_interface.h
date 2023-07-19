#ifndef UWB_LOCALIZATION_H
#define UWB_LOCALIZATION_H
#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <stdio.h>

// ros
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "range_localization/basic_function.hpp"
#include "range_localization/uwb_localization.hpp"
// #include "range_localization/uwb_loc_init.hpp"

#include "rtls_uwb_sensor/uwbs.h"
#include "rtls_uwb_sensor/uwb.h"
#include "nlink_parser/LinktrackNodeframe3.h"
#include "nlink_parser/LinktrackNode2.h"

// using std::placeholders::_1;
// using namespace std::chrono_literals;

namespace uavos {

class UWB_Localization
{
public:
    UWB_Localization(ros::NodeHandle &nh);
    
    ~UWB_Localization(){
        // sp.close();
    }

    ros::NodeHandle m_nh;
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_pressure_sub;
    ros::Subscriber m_motor_speed_sub;
    ros::Subscriber m_range_sub;
    ros::Timer m_print_timer;
    ros::Timer m_range_timer;
    ros::Publisher pose_predict_pub;
    ros::Publisher pose_filter_pub;
    ros::Publisher pubPath;
    nav_msgs::Path globalPath;

    int m_slam_fps;
    bool m_is_fix_fps;
    bool m_is_initialize_with_ceres;
    bool m_use_pressure_height;
    bool m_use_imu;
    bool m_use_laser_height;

    std::string m_vehicle_name;
    std::string  m_uwb_sensor_type;
    boost::shared_ptr<uavos::UWB_Mobile> m_p_mobile;

    double m_pressure = 0.0;
    int m_callback_len;
    
    std::string m_imu_sub_topic;
    std::string m_range_sub_topic;
    std::string m_pressure_sub_topic;
    std::string m_motor_speed_sub_topic;
    std::string m_height_sub_topic;
    std::string m_uwb_port;
    std::string m_frame_id;


    std::vector<std::string> m_anchor_list;
    std::map<std::string, boost::shared_ptr<uavos::UWB_Anchor> > m_anchor_map;

    Eigen::Vector3d m_gps_init;

    double m_motor_speed = 0;
    bool m_motor_called = false;


    void createUWBMobile(const int mobile_id);

    void imuCallback(const sensor_msgs::Imu & msg);

    void pressureCallback(const std_msgs::Float32 & msg);
    void motorSpeedCallback(const std_msgs::Float32MultiArray & msg);
    void heightCallback(const std_msgs::Float32 & msg);
    void SkpHeightCallback(const geometry_msgs::Vector3Stamped & msg);

    void rangeCallback();
    void rangeOfRICallback1(const rtls_uwb_sensor::uwbs & msg);
    void rangeOfRICallback2(const nlink_parser::LinktrackNodeframe3 & msg);

    void onlyRangeCallback(const rtls_uwb_sensor::uwbs & msg);


    // void printLocalizationCallback(const ros::TimerEvent& event); 
    void printLocalizationCallback(double stamp);

};



}


#endif // UWB_LOCALIZATION_H

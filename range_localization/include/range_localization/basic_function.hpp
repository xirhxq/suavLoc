#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdio.h>


#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <ceres/ceres.h>

// boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include  <boost/array.hpp>

namespace uavos{

// the small delta is to ensure that 180 degree is remain 180, instead of -180.
//#define INPI(angle)		(angle -= floor((angle+M_PI-0.0000000001)/(2*M_PI))*2*M_PI)
//#define INPI(angle)		(angle -= floor((angle+M_PI)/(2*M_PI))*2*M_PI)
struct HEADER
{
    double stamp; // ros::Time::now().toSec();
};
struct POSE
{
    Eigen::Vector3d position = Eigen::Vector3d(0,0,0);
    Eigen::Quaterniond orientation = Eigen::Quaterniond(1,0,0,0);
};
struct TWIST
{
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
};
struct NavigationState
{
    HEADER header;
    POSE pose;
    TWIST velocity;
    TWIST acceleration;
};
struct RangeInfo
{
    double stamp; // s
    std::string nodeId;
    std::string responderId; 
    double precisionRangeErrEst; // m
    float precisionRangeM; // m
    double rangeMm;
    double rangeErrorEstimate;
    double rssi_fp;
    double rssi_rx;
};
struct RangeInfoVec
{
    double stamp; // s
    std::vector<RangeInfo> RangeInfos;
};
struct NodeParameters
{
    double z_damping_factor;
    double kalman_sigma_a;
    double Q_scale;
    double R_scale;
    double tao_acc_sqrt;
    double tao_bias_sqrt;
    double sigma_pressure;
    Eigen::Vector3d position;
};


template<typename T>
inline std::string num2str(const T src){
    std::stringstream ss;
    ss << src;
    return ss.str();
}

template<typename T>
inline T INPI(const T src){
    T dst = src;

    // the small delta is to ensure that 180 degree is remain 180, instead of -180.
    dst -= floor((dst+M_PI-0.0000000001)/(2*M_PI))*2*M_PI;
    return dst;
}




inline NavigationState getZeroNavigationState(){
    NavigationState state;
    state.header.stamp = -1.0;

    state.pose.position.x() = 0;
    state.pose.position.y() = 0;
    state.pose.position.z() = 0;
    state.pose.orientation.x() = 0;
    state.pose.orientation.y() = 0;
    state.pose.orientation.z() = 0;
    state.pose.orientation.w() = 1;

    state.velocity.linear.x()=0;
    state.velocity.linear.y()=0;
    state.velocity.linear.z()=0;
    state.velocity.angular.x()=0;
    state.velocity.angular.y()=0;
    state.velocity.angular.z()=0;

    state.acceleration.linear.x()=0;
    state.acceleration.linear.y()=0;
    state.acceleration.linear.z()=0;
    state.acceleration.angular.x()=0;
    state.acceleration.angular.y()=0;
    state.acceleration.angular.z()=0;

    return state;
}
inline void setZeroNavigationState(NavigationState& state){
    state.header.stamp = -1.0;

    state.pose.position.x() = 0;
    state.pose.position.y() = 0;
    state.pose.position.z() = 0;
    state.pose.orientation.x() = 0;
    state.pose.orientation.y() = 0;
    state.pose.orientation.z() = 0;
    state.pose.orientation.w() = 1;

    state.velocity.linear.x() = 0;
    state.velocity.linear.y() = 0;
    state.velocity.linear.z() = 0;
    state.velocity.angular.x() = 0;
    state.velocity.angular.y() = 0;
    state.velocity.angular.z() = 0;

    state.acceleration.linear.x() = 0;
    state.acceleration.linear.y() = 0;
    state.acceleration.linear.z() = 0;
    state.acceleration.angular.x() = 0;
    state.acceleration.angular.y() = 0;
    state.acceleration.angular.z() = 0;
}
inline void setZeroVelocityAccleration4NavigationState(NavigationState& state){
    state.header.stamp = -1.0;

    state.velocity.linear.x() = 0;
    state.velocity.linear.y() = 0;
    state.velocity.linear.z() = 0;
    state.velocity.angular.x() = 0;
    state.velocity.angular.y() = 0;
    state.velocity.angular.z() = 0;

    state.acceleration.linear.x() = 0;
    state.acceleration.linear.y() = 0;
    state.acceleration.linear.z() = 0;
    state.acceleration.angular.x() = 0;
    state.acceleration.angular.y() = 0;
    state.acceleration.angular.z() = 0;
}
inline void setZeroAccleration4NavigationState(NavigationState& state){
    state.header.stamp = -1.0;

    state.acceleration.linear.x() = 0;
    state.acceleration.linear.y() = 0;
    state.acceleration.linear.z() = 0;
    state.acceleration.angular.x() = 0;
    state.acceleration.angular.y() = 0;
    state.acceleration.angular.z() = 0;
}

// vector to NavigationState

// vector arithematic
template <typename T, const size_t N>
inline boost::array<T,N> vectorSubtraction(const boost::array<T,N> &a, const boost::array<T,N> &b){
    boost::array<T,N> dst;
    for(int i=0;i<a.size();++i){
        dst.at(i) = ( a.at(i)-b.at(i) );
    }
    return dst;
}
template <typename T, const size_t N>
inline boost::array<T,N> vectorAddition(const boost::array<T,N> &a, const boost::array<T,N> &b){
    boost::array<T,N> dst;
    for(int i=0;i<a.size();++i){
        dst.at(i) = ( a.at(i)+b.at(i) );
    }
    return dst;
}
template <typename T, size_t N>
inline void setZeroVector(boost::array<T,N> &a){
    for(int i=0;i<a.size();++i){
        a.at(i) = 0;
    }
}


template <typename T>
inline std::vector<T> vectorSubtraction(const std::vector<T> &a, const std::vector<T> &b){
    std::vector<T> dst;
    for(int i=0;i<a.size();++i){
        dst.push_back( a.at(i)-b.at(i) );
    }
    return dst;
}
template <typename T>
inline std::vector<T> vectorAddition(const std::vector<T> &a, const std::vector<T> &b){
    std::vector<T> dst;
    for(int i=0;i<a.size();++i){
        dst.push_back( a.at(i)+b.at(i) );
    }
    return dst;
}
template <typename T>
inline void setZeroVector(std::vector<T> &a){
    for(int i=0;i<a.size();++i){
        a.at(i) = 0;
    }
}


inline void quaternion2RPY(const Eigen::Quaterniond q, double &roll, double &pitch, double &yaw){
    Eigen::Vector3d eular = q.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw = INPI<double>(eular[2]);
    pitch = INPI<double>(eular[1]);
    roll = INPI<double>(eular[0]);
}


inline double constrain(double a, double lb, double ub)
{
    if(a < lb) return lb;
    else if (a > ub) return ub;
    else return a; 
}

inline double statistics_deviation(const std::deque<double> vec_double)
{
    double deviation = 0;
    double mean = 0;
    auto it = vec_double.begin();
    while(it != vec_double.end())
    {
        mean += *it;
        it++;
    }
    mean = mean/vec_double.size();
    printf("mean: %lf ", mean);
    it = vec_double.begin();
    while(it != vec_double.end())
    {
        deviation += (*it - mean)*(*it - mean);
        it++;
    }
    deviation = sqrt(deviation/vec_double.size());
    printf("deviation: %lf ", deviation);
    return deviation;
}

inline double statistics_average(const std::deque<double> vec_double)
{
    double deviation = 0;
    double mean = 0;
    auto it = vec_double.begin();
    while(it != vec_double.end())
    {
        mean += *it;
        it++;
    }
    mean = mean/vec_double.size();

    return mean;
}

inline bool check_height_stable(const double height, double &ave_height)
{
    static std::deque<double> vec_height;
    vec_height.push_back(height);
    if(vec_height.size()>100)
    {
        if(statistics_deviation(vec_height) < 0.1)
        {
            ave_height = statistics_average(vec_height); 
            printf("uwb height unstable! average height %lf", ave_height);
            return true;
        }
        vec_height.pop_front();
        printf("uwb height unstable!");
        return false;
    }
}

inline bool check_height_stable(std::deque<double> &vec_height, double &ave_height)
{
    if(vec_height.size()>100)
    {
        if(statistics_deviation(vec_height) < 0.2)
        {
            ave_height = statistics_average(vec_height); 
            printf("pressure height stable! average delta height between uwb %lf", ave_height);
            vec_height.clear();
            return true;
        }
        vec_height.pop_front();
        printf("uwb height unstable!");
        return false;
    }
    return false;
}

}

#endif //BASIC_FUNCTION_H_

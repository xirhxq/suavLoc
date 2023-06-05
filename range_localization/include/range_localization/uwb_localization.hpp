#ifndef UWB_NODE_H
#define UWB_NODE_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <queue>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>
#include <string>

#include "range_localization/basic_function.hpp"

// #define gt

namespace uavos{

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
#define DEBUG_FILE_DIR(name)     (std::string(std::string(ROOT_DIR) + "Log/"+ name))

// -----------------------------
struct comp{
    template <typename T>
    inline bool operator()(const T& a, const T& b) const{
        return (a<b);
    }
};



template <typename T>
double robustAverage(std::vector<T> array){
    // the paramter is copied, so that the original data will not be affected.
    const size_t cut_length = 2;

    if(array.size()<=2*cut_length){
        return 0;
    }

    double sum = 0;
    std::sort(array.begin(), array.end(), comp());
    for(size_t i=cut_length;i<array.size()-cut_length;++i){
        sum+=array.at(i);
    }

    return sum / (array.size()-2*cut_length);

}

template <typename T>
bool medianAndVariance(std::vector<T> array, double& median, double& variance){
    if(array.size()==0){
        return false;
    }

    // get median
    std::sort(array.begin(), array.end(), comp());
    median = array.at(std::floor(array.size()/2));

    // get mean
    double sum = 0;
    for(int i=0;i<array.size();++i){
        sum+=array.at(i);
    }
    double mean = sum / array.size();


    // get variance
    double variance_sum = 0;
    for(int i=0;i<array.size();++i){
        double res = array.at(i)-mean;
        variance_sum += res*res;
    }
    variance = variance_sum / array.size();

    return true;
}

// -----------------------------


class UWB_Node{
public:

    explicit UWB_Node(const std::string node_id);

protected:

    std::string m_node_id;
    Eigen::Vector3d m_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_velocity = Eigen::Vector3d::Zero();


public:
    inline int get_id(std::string id_str){
        int sz = id_str.length();
        if (id_str[sz - 2] >= '0' && id_str[sz - 2] <= '9'){
            return (id_str[sz - 2] - '0') * 10 + (id_str[sz - 1] - '0');
        }
        else{
            return id_str[sz - 1] - '0';
        }
    }
    inline std::string getNodeId() const{
        return m_node_id;
    }
    inline Eigen::Vector3d getPosition() const{
        return m_position;
    }
    inline Eigen::Vector3d* getPositionPtr(){
        return &m_position;
    }
    inline Eigen::Vector3d getVelocity() const{
        return m_velocity;
    }


    inline void setPosition(const double x, const double y, const double z){
        m_position.x() = x;
        m_position.y() = y;
        m_position.z() = z;
    }
    inline void setVelocity(const double vx, const double vy, const double vz){
        m_velocity.x() = vx;
        m_velocity.y() = vy;
        m_velocity.z() = vz;
    }

};


class UWB_Anchor : public UWB_Node{
public:
    explicit UWB_Anchor(const std::string node_id, const Eigen::Vector3d p, const double t);

public:

    // std::map<int, std::vector<double> > m_range_map;
    // void insertRangeInfo(const RangeInfo& msg);
    // double getAverageRangeToNode(const int anchor_id);
    // bool getMedianAndVarianceToNode(const int anchor_id, double& median, double& variance);
    
    double m_last_update_time;
    Eigen::Matrix3d m_cov_position;
    std::deque<double> m_time_pos_que;
    std::deque<Eigen::Vector3d> m_pos_que;

    void updatePosition(const Eigen::Vector3d position, const double t)
    {
        m_last_update_time = t;
        m_position = position;
        m_pos_que.push_back(position);
        m_time_pos_que.push_back(t);
        if(m_time_pos_que.size() > 100)
        {
            m_pos_que.pop_front();
            m_time_pos_que.pop_front();
        }
    }

    Eigen::Matrix3d getPosCov()
    {
        Eigen::Vector3d t_s(0,0,0);
        Eigen::Matrix3d cov= Eigen::Matrix3d::Identity();
        for(size_t i = 0; i < m_time_pos_que.size(); i ++)
        { 
            t_s += m_pos_que.at(i);
        }
        t_s /= m_time_pos_que.size();
        cov(0,0) = t_s(0);
        cov(1,1) = t_s(1);
        cov(2,2) = t_s(2);
        return m_cov_position;
    }

    double getUpdateTime()
    {
        return m_last_update_time;
    }

    inline void printPosition(){
        std::cout<<m_node_id<<": "<<m_position.x()<<", "<<m_position.y()<<", "<<m_position.z()<<std::endl;
    }
};


class UWB_Mobile : public UWB_Node{
public:
    explicit UWB_Mobile(const std::string node_id);
    
    
    double m_last_predict_time;
    double m_last_update_time;

    bool m_pressure_is_update = false;
    bool m_imu_is_inited = false;
    bool m_imu_called = false;
    bool m_range_called = false;
    bool m_switch_pressure_on = false;
    
    // anchor 
    std::map<std::string, boost::shared_ptr<UWB_Anchor> > m_anchor_map;
    std::map<std::string, double > m_anchor_bias;

    // variables for filter
    Eigen::Vector3d m_acc_bias;
    double m_h_bias = 0; // delta h between height and uwb height
    Eigen::Vector4d m_r_bias;
    // double m_state_dimension = ; 
    Eigen::MatrixXd m_state_covariance;
    
    // acc input 
    NavigationState m_uav_state;

    // store log
    bool m_log_residual = true;
    std::ofstream logResidualFile, log_predict_state, log_update_state, log_update_pressure;
    ~UWB_Mobile()
    {
        if(m_log_residual)
        {
            logResidualFile.close();
            log_predict_state.close();
            log_update_state.close();
            log_update_pressure.close();
        }
    }
    
private:

    // stores last correct/accepted measurements
    std::map<int, RangeInfo> m_ranges_to_anchors;
    
    // provided by parameter server
    double m_kalman_sigma_a;
    double m_snr_threshold;
    double m_innovation_threshold;
    double m_tao_acc_sqrt;
    double m_tao_bias_sqrt;
    double m_z_damping_factor;
    double m_Q_scale;
    double m_R_scale;
    double m_sigma_pressure;

    Eigen::Matrix3d m_DOP;




public:

    inline Eigen::Vector3d getAccelaration() const{
        return m_uav_state.acceleration.linear;
    }
    inline Eigen::Quaterniond getOritation() const{
        return m_uav_state.pose.orientation;
    }

    inline double getUpdateTime() const{
        return m_last_update_time;
    }
    inline double getPredictTime() const{
        return m_last_predict_time;
    }
    
    inline void setAccelerationBias(const double ax_bias, const double ay_bias, const double az_bias){
        m_acc_bias.x() = ax_bias;
        m_acc_bias.y() = ay_bias;
        m_acc_bias.z() = az_bias;
    }
    inline Eigen::MatrixXd getStateCovariance() const{
        return m_state_covariance;
    }
    inline void setStateCovariance(const Eigen::MatrixXd P){
        m_state_covariance = P;
    }

    void ekf_predict(const Eigen::Quaterniond imu_ori, const Eigen::Vector3d imu_acc, const double t_imu);
    bool ekf_update_loosely(const RangeInfoVec &vec_range_info);
    bool ekf_update_tightly(const RangeInfoVec &vec_range_info);
    Eigen::Vector3d get_LQ_result(const RangeInfoVec &vec_range_info);
    Eigen::Matrix<double, 4, 3> get_anchors_position();
    bool ekf_update_pressure(const double &fluid_pressure, const double &t_pres);
    bool ekf_update_pressure_bias(const double &fluid_pressure, const double &t_pres);

    inline bool getAnchorPosition(const std::string anchor_id, Eigen::Vector3cd anchor_position) const {
        if(m_anchor_map.find(anchor_id)!=m_anchor_map.end()){
            anchor_position = m_anchor_map.find(anchor_id)->second->getPosition();
            return true;
        } else {
            return false;
        }
    }
    inline int getAnchorNumber() const{
        return m_anchor_map.size();
    }


    bool acc_body_to_NWU(const Eigen::Quaterniond q_b_to_w, const Eigen::Vector3d acc_body, NavigationState &uav_state);
    
    void simpleInitializeEKF(NodeParameters initParameter);


    // print
    inline void printPosition(){
        std::cout<<m_node_id<<":"<<std::endl;
        std::cout<<"pose: "<<m_position.x()<<", "<<m_position.y()<<", "<<m_position.z()
                <<"   cov: "<<m_state_covariance(0,0)<<", "<<m_state_covariance(3,3)<<", "<<m_state_covariance(6,6)
                <<std::endl;
    }
    inline void printVelocity(){
        std::cout<<"velo: "<<m_velocity.x()<<", "<<m_velocity.y()<<", "<<m_velocity.z()
                <<"   cov: "<<m_state_covariance(1,1)<<", "<<m_state_covariance(4,4)<<", "<<m_state_covariance(7,7)
                <<std::endl;
    }
    inline void printAccelerationBias(){
        std::cout<<"acc bias: "<<m_acc_bias.x()<<", "<<m_acc_bias.y()<<", "<<m_acc_bias.z()
                    <<"   cov: "<<m_state_covariance(2,2)<<", "<<m_state_covariance(5,5)<<", "<<m_state_covariance(8,8)
                    <<std::endl;
    }
};
}

#endif // UWB_NODE_H

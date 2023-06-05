#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <deque>
#include "Thirdparty/asyncserial/examples/heightSensor/include/skp40_height_sensor.h"

double averaging_filter(const double height);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skp40_height_sensor");
    ros::NodeHandle nh("~");
    
    std::string m_serial_port;
    nh.param<std::string>("serial_port", m_serial_port, "/dev/ttyUSB0");
    std::cout << m_serial_port << std::endl;
    int m_serial_baudrate;
    nh.param<int>("serial_baudrate", m_serial_baudrate, 115200);

    ros::Publisher height_publisher = nh.advertise<geometry_msgs::Vector3Stamped>("/skp40_height_sensor/data", 1);

    ROS_INFO("%s,%d",m_serial_port,m_serial_baudrate);

    height_sensor sensor(m_serial_port, m_serial_baudrate);;
    sensor.check_sensor_state();

    while(ros::ok())
    { 
        geometry_msgs::Vector3Stamped height_msgs;
        height_msgs.header.stamp = ros::Time::now();
        double raw_data = sensor.get_height()/ 1000; // mm - > m
        height_msgs.vector.x = raw_data ; // m
        height_msgs.vector.y = averaging_filter(raw_data); // m
        height_publisher.publish(height_msgs);
    }
    return 0;
}

double averaging_filter(const double height)
{
    static std::deque<double> vec_height;
    vec_height.push_back(height);
    if(vec_height.size()>10)
    {
        double mean = 0;
        auto it = vec_height.begin();
        while(it != vec_height.end())
        {
            mean += *it;
            it++;
        }
        mean = mean/vec_height.size();
        vec_height.pop_front();
        return mean;
    }
    else
    {
        return height;
    }
}

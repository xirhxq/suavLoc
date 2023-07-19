
#include <iostream>
#include <string>
#include <string.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h> 

#include "rtls_uwb_sensor/uwb.h"
#include "rtls_uwb_sensor/uwbs.h"
#include "tic_toc.h"

using namespace std;


int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "uwb_sensor");
    ros::NodeHandle nh;
    ros::Publisher uwb_publisher = nh.advertise<rtls_uwb_sensor::uwbs>("/uwb/data", 1);

    //创建一个serial类
    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(11);
    sp.setPort("/dev/ttyUSB2");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    TicToc time_freq;
    time_freq.tic();
    while(ros::ok())
    {
        // printf("loop: %lf\n",time_freq.toc());
        //获取缓冲区内的字节数
        size_t data_size = sp.available(); //读取指定长度 
        
        if(data_size>200)
        {            
            std::string strtmp;
            sp.readline(strtmp);
            std::cout<<"received: "<<strtmp<<std::endl;
            auto  n = strtmp.find("mc"); //返回的是T2出现的位置 
            if (n != std::string::npos && n == 0)
            {
                printf("uwb: %lf\n",time_freq.toc());
                rtls_uwb_sensor::uwbs uwbs_data;
                char chartmp[200] = {0};
                strtmp.copy(chartmp, strtmp.length(), 0);

                int aid, tid, lnum, seq, mask, range[4];
                int rangetime;
                char c, type;
                double time;

                int n = sscanf((char*)chartmp,"m%c %x %x %x %x %x %x %x %x %c%d:%d", &type, &mask, &range[0], &range[1], &range[2], &range[3], &lnum, &seq, &rangetime, &c, &tid, &aid);
                printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)\nrange[3]=%d(mm)\r\n", mask,range[0], range[1], range[2], range[3]);

                
                uwbs_data.header.stamp = ros::Time::now();
                int num_anchor = 4;
                for(int i = 0; i < num_anchor; i++)
                {
                    if(range[i] > 700000 || range[i] < 0 || range[i] == NULL)
                    {
                        continue;
                    }     
                    rtls_uwb_sensor::uwb uwb_data;
                    uwb_data.responderId = std::string("anchor_") + std::to_string(i);
                    uwb_data.precisionRangeM = range[i] * 1.0 / 1000.0;
                    uwb_data.precisionRangeErrEst = 0.1;
                    uwbs_data.uwb_vec.push_back(uwb_data);
                }
                uwb_publisher.publish(uwbs_data);
            }            
        }    
    }
    //关闭串口
    sp.close();
    return 0;
}

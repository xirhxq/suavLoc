#pragma once
#include <iostream>
#include <string>
#include <sstream>
int get_serial_io(const std::string key_words, std::string &serial_port)   
{
/**
 * @brief 
 * [3.843702] usb 1-4.3: xsens_mt converter now attached to ttyUSB0
 * [3.845132] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB1
 * [3.845837] usb 1-2: ch341-uart converter now attached to ttyUSB2
 */


    char *buf_line;  
    size_t len = 1024;
    FILE *ptr = NULL; 
    // std::string cmd = "dmesg | grep tty";  // 
    std::string cmd = "ls -l  /dev/serial/by-id/";  //https://blog.csdn.net/weixin_40378209/article/details/121727386

    std::cout << "\033[31m" << "Auot scan the nooploop serial port" << "\033[0m" << std::endl;
    std::cout << "\033[31m" << key_words << "\033[0m" << std::endl;
    if((ptr=popen(cmd.c_str(), "r"))!=NULL)   
    {
    	char ret[1024] = {0};
    	
    	int nread = fread(ret,1,1024,ptr);
    	printf("%d,%s",nread,ret);
    	
    	std::stringstream ss(ret);
    	//std::cout << ss.c << std::endl;
    	
    	std::vector<std::string> tokens;
    	std::string token;
    	while(std::getline(ss,token,'\n'))
    	{
    	    tokens.push_back(token);
    	}
        //while(std::getline(ret,line)!= -1)  
        //while(ptr->getline(&buf_line, len)!= -1)  
        int index = 0;
        std::cout << "tokens size : " << tokens.size() << std::endl;
        for(int idx = 0; idx < tokens.size(); idx ++){
           std::string line = tokens[idx];
           std::cout << "Finding In " << "\033[31m" << line << "\033[0m" << std::endl;
           if(line.find(key_words) != std::string::npos)
           {
                if(line.find("ttyUSB") != std::string::npos)
                {
                    int pos = line.find("ttyUSB");
                    std::cout << "line[pos+6]: " << line[pos+6] << std::endl;
                    int usb_port_num = line[pos+6] - '0';
                    pclose(ptr);   
                    ptr = NULL;   
                    serial_port = "/dev/ttyUSB" + std::to_string(usb_port_num);
                    // green cout
                    std::cout << "\033[32m" << "Find the serial port: " << serial_port << "\033[0m" << std::endl;
                    return usb_port_num;
                }
                else
                {
                    std::cout << "\033[31m" << "NOT FIND ttyUSB IN " << line << "\033[0m" << std::endl;
                }
           }
           else
           {
                std::cout << "\033[31m" << "NOT FIND "<< key_words << " IN " << line << "\033[0m" << std::endl;
           }   
              
        }   
        pclose(ptr);   
        ptr = NULL;   
    }   
    else  
    {   
        std::cout << "\033[31m" << "popen " << cmd << " error" << "\033[0m" << std::endl;
    }
    std::cout << "Ending" << std::endl;
    return -1;
}

int get_serial_io_by_path(const std::string key_words, std::string &serial_port)   
{
/**
 * @brief 
 * [3.843702] usb 1-4.3: xsens_mt converter now attached to ttyUSB0
 * [3.845132] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB1
 * [3.845837] usb 1-2: ch341-uart converter now attached to ttyUSB2
 */


    char *buf_line;  
    size_t len = 1024;
    FILE *ptr = NULL; 
    // std::string cmd = "dmesg | grep tty";  // 
    std::string cmd = "ls -l  /dev/serial/by-path/";  //https://blog.csdn.net/weixin_40378209/article/details/121727386

    std::cout << "\033[31m" << "Auot scan the nooploop serial port" << "\033[0m" << std::endl;
    if((ptr=popen(cmd.c_str(), "r"))!=NULL)   
    {
        while(getline(&buf_line, &len, ptr)!= -1)  
        {
           std::stringstream ss;
           ss << buf_line; 
           std::string line = ss.str();
           if(line.find(key_words) != std::string::npos)
           {
                if(line.find("ttyUSB") != std::string::npos)
                {
                    int pos = line.find("ttyUSB");
                    std::cout << "line[pos+6]: " << line[pos+6] << std::endl;
                    int usb_port_num = line[pos+6] - '0';
                    pclose(ptr);   
                    ptr = NULL;   
                    serial_port = "/dev/ttyUSB" + std::to_string(usb_port_num);
                    std::cout << "\033[32m" << "Find the serial port: " << serial_port << "\033[0m" << std::endl;
                    return usb_port_num;
                }
                else
                {
                    std::cout << "\033[31m" << "NOT FIND ttyUSB IN " << line << "\033[0m" << std::endl;
                }
           }
           else
           {
                std::cout << "\033[31m" << "NOT FIND "<< key_words << " IN " << line << "\033[0m" << std::endl;
           }   
              
        }   
        pclose(ptr);   
        ptr = NULL;   
    }   
    else  
    {   
        // printf("popen %s error", cmd.c_str());   
        std::cout << "\033[31m" << "popen " << cmd << " error" << "\033[0m" << std::endl;
    }
    return -1;
}

//
// Created by ljn on 23-5-8.
//

#ifndef ASYNCSERIAL_TIMER_H
#define ASYNCSERIAL_TIMER_H

#include <chrono>
#include <ctime>

class Timer {

protected:
    std::chrono::time_point<std::chrono::system_clock>  end;
    std::chrono::time_point<std::chrono::system_clock> start;
public:
    Timer(){};
    ~Timer(){};
    void tic()
            {
        start = std::chrono::system_clock::now();
    }
    double toc()
            {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count();
    }
    double tictoc()
    {
        static std::chrono::time_point<std::chrono::system_clock> global_start = std::chrono::system_clock::now();
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - global_start;
        return elapsed_seconds.count();
    }
};
#endif//ASYNCSERIAL_TIMER_H

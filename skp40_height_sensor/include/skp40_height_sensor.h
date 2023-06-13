#include <stdlib.h>

#include <algorithm>
#include <future>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "Thirdparty/asyncserial/include/asyncserial/AsyncSerial.h"
#include "timer.h"


class height_sensor
{
public:
    height_sensor(std::string serial_port, int serial_baudrate);
    ~height_sensor();
    double get_height();
    bool check_sensor_state();
    bool checked_sensor_state = false;
protected:
    AsyncSerial serial;
    Timer timer;
    void print_data(std::vector<uint8_t> const received_data);
    uint8_t get_crc8(std::vector<uint8_t> data_in);
};
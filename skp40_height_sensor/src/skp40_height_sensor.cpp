#include "skp40_height_sensor.h"


uint8_t height_sensor::get_crc8(std::vector<uint8_t> data_in) {
    uint8_t crc = 0x00;
    uint8_t i;
    for (int j = 1; j < 6; j++) {
        crc ^= data_in[j];// key + value -> 2-6 -> 1-5
        for (i = 8; i > 0; i--) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}
void height_sensor::print_data(std::vector<uint8_t> const received_data) {
    std::cout << "[" << timer.tictoc() << "]" << " ";
    std::for_each(std::begin(received_data),
                  std::end(received_data),
                  [](uint8_t const ch) { std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<size_t>(ch) << " "; });
    std::cout << std::endl;
}

height_sensor::height_sensor(std::string serial_port, int serial_baudrate)
{
    serial.open(serial_port, serial_baudrate);
}
height_sensor::~height_sensor()
{
    serial.close();
}

bool height_sensor::check_sensor_state()
{
    while(!checked_sensor_state)
    {
        std::vector<uint8_t> received_data;
        std::vector<uint8_t> const stop_measure = {0x55, 0x06, 0x00, 0x00, 0x00, 0x00, 0x88, 0xAA};//停止测量
        serial.transmit(stop_measure);
        sleep(1);
        std::vector<uint8_t> const get_sensor_setting = {0x55, 0x01, 0x00, 0x00, 0x00, 0x00, 0xD3, 0xAA};//获取设备信息，第二帧　55 01 AA(数据格式) BB(测量模式) CC CC(测量频率) JY AA
        serial.transmit(get_sensor_setting);
        sleep(1);
        received_data = serial.receive(8).get();
        print_data(received_data);
        received_data = serial.receive(8).get();
        print_data(received_data);

        if(received_data[0] != 0x55) // flush the buffer for 55 as the first byte
        {
            auto idx = std::find(received_data.begin(), received_data.end(), 0x55);
            if (idx != received_data.end())
            {
                int idx_from_the_first = std::distance(received_data.begin(), idx);
                received_data = serial.receive(idx_from_the_first).get();
                print_data(received_data);
            }
        }
        if(received_data[1] != 0x01) // 获取设备信息(0x01)，否则失败
        {
            sleep(1);
            continue ;
        }
        else if (received_data[1] == 0x01)
        {
            // parameters
            std::vector<uint8_t> const data_mode = {0x55, 0x04, 0x00, 0x00, 0x00, 0x01, 0x2E, 0xAA};//设置数据格式（0x04） 字节格式
            std::vector<uint8_t> const work_mode = {0x55, 0x0D, 0x00, 0x00, 0x00, 0x00, 0xF2, 0xAA};//设置测量模式（连续测量-开机启动）
            int work_rate = 200;// 1-4500Hz
            std::cout << std::setw(2) << std::setfill('0') << std::hex << work_rate << std::endl;
            std::vector<uint8_t> data_hz = {0x55, 0x03, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xAA};//设置测量频率
            //  data_hz[2] = ((baudrate & 0xFF000000)>>24);
            //  data_hz[3] = ((baudrate & 0x00FF0000)>>16);
            data_hz[4] = ((work_rate & 0x0000FF00) >> 8);
            data_hz[5] = ((work_rate & 0x000000FF));
            data_hz[6] = get_crc8(data_hz);
            std::cout << "the rate hex is: ";
            print_data(data_hz);

            print_data(received_data);
            
            if((received_data[2] == data_mode[5]) && (received_data[3] == work_mode[5]) && (received_data[4] == data_hz[4]) && (received_data[5] == data_hz[5]))
            {
                std::cout << "setting is correct" << std::endl;
                checked_sensor_state = true;
                break;
            }
            else
            {
                // set parameters
                serial.transmit(data_mode);
                sleep(1);
                serial.transmit(work_mode);
                sleep(1);
                serial.transmit(data_hz);
                sleep(1);
                // save for loaded automatically at next boot
                std::vector<uint8_t> const save_setting = {0x55, 0x08, 0x00, 0x00, 0x00, 0x00, 0x3E, 0xAA};//保存设置,返回一帧数据（55 08 AA AA AA AA JY AA，AA全部为0表成功保存，其他为失败）
                serial.transmit(save_setting);
                sleep(1);
                received_data = serial.receive(8).get();
                std::cout << "setting finished result is " << std::endl;
                print_data(received_data);
                if (received_data[1] == 0x08 && received_data[2] == 0x00 && received_data[3] == 0x00 && received_data[4] == 0x00  && received_data[5] == 0x00)
                {
                    checked_sensor_state = true;
                }
            }
        }
    }
    //  std::vector<uint8_t> set_baud_rate = {0x55, 0x12, 0x00, 0x00, 0x00, 0x10, 0xFF, 0xAA};//设置波特率 0x0F:460800  0x10:921600
    //  set_baud_rate[6] = get_crc8(set_baud_rate);
    //  serial.transmit(set_baud_rate);
    std::vector<uint8_t> const begin_measure = {0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xAA};//启动测量
    serial.transmit(begin_measure);
    sleep(1);
    return true;
}

double height_sensor::get_height()
{
//    std::vector<uint8_t> future = {0x55, 0x07, 0x00, 0x52, 0x50, 0x01, 0x57, 0xAA};//test
    std::cout << "begin to receive" << std::endl;
    std::vector<uint8_t> future = serial.receive(8).get();
    print_data(future);
	if(future[0] != 0x55) // flush the buffer for 55 as the first byte
    {
        auto idx = std::find(future.begin(), future.end(), 0x55);
        if (idx != future.end())
        {
            int idx_from_the_first = std::distance(future.begin(), idx);
            future = serial.receive(idx_from_the_first).get();
            print_data(future);
        }
        return 0;
    }
    if(future[1] == 0x07)
    {
        switch (future[2]) {
            case 0x00:
            {
                // std::cout << "[" << timer.tictoc() << "]" <<"系统正常 ";
                if (future[6] == get_crc8(future))
                {
                    // std::cout << "CRC校验成功 ";
                }
                else
                {
                    // std::cout << "CRC校验失败 ";
                }
                int height = 0;
                height += (future[3] << 16);
                height += (future[4] << 8);
                height += (future[5]);
                // std::cout << "高度：" << std::dec << height << "mm" << std::endl;
                return height;
            }
            case 0x01:
                std::cout << "[" << timer.tictoc() << "]" <<"信号过弱" << std::endl;
                break ;
            case 0x02:
                std::cout << "[" << timer.tictoc() << "]" <<"信号过强" << std::endl;
                break ;
            case 0x03:
                std::cout << "[" << timer.tictoc() << "]" <<"超出量程" << std::endl;
                break ;
            case 0x04:
                std::cout << "[" << timer.tictoc() << "]" <<"系统错误" << std::endl;
                break ;
        }
    }
}

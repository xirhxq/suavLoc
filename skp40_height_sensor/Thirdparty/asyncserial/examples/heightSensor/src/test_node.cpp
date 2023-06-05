#include "skp40_height_sensor.h"

int main(int argc, char **argv)
{
    height_sensor sensor("/dev/ttyUSB0", 115200);;
    sensor.check_sensor_state();
    while (true)
    {
        sensor.get_height();
    }
    return 0;
}
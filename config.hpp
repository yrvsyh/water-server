#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

class Config
{
public:
    static std::string SENSOR_PATH;
    static std::string CAMERA_PATH;
    static std::string SENSOR_ADDR;
    static uint16_t SENSOR_PORT;
    static uint16_t LISTEN_PORT;
    static std::string RTMP_URL;
};

#endif // CONFIG_HPP

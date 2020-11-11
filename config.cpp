#include "config.hpp"

std::string Config::SENSOR_PATH = "/dev/ttyUSB0";
std::string Config::CAMERA_PATH = "/dev/video0";
std::string Config::SENSOR_ADDR = "127.0.0.1";
uint16_t Config::SENSOR_PORT = 9999;
uint16_t Config::LISTEN_PORT = 1234;
std::string Config::RTMP_URL = "rtmp://127.0.0.1:1935/live";

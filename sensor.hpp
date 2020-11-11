#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "muduo/base/Logging.h"
#include "muduo/base/Timestamp.h"
#include "muduo/net/Buffer.h"
#include "muduo/net/Channel.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/InetAddress.h"
#include "muduo/net/TcpClient.h"

#include "serialport.hpp"

#include <memory>

class Sensor
{
public:
    Sensor(muduo::net::EventLoop *loop, std::string ttyPath, std::string addr,
           uint16_t port);
    ~Sensor();

private:
    void onMessage(muduo::Timestamp);
    void sendSensorData(std::string data);

private:
    muduo::net::EventLoop *loop_;
    SerialPort serialPort_;
    muduo::net::InetAddress srvAddr_;
    muduo::net::TcpClient client_;
    muduo::net::Buffer buffer_;
    std::unique_ptr<muduo::net::Channel> serialChannel_;
};

#endif // SENSOR_HPP

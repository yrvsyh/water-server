#ifndef WATERSERVER_HPP
#define WATERSERVER_HPP

#include "muduo/base/ThreadPool.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/TcpServer.h"

#include "camera.hpp"
#include "sensor.hpp"
#include "slider.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class WaterServer
{

public:
    WaterServer(muduo::net::EventLoop *loop, std::string ttyPath, std::string sensorAddr,
                uint16_t sensorPort, std::string camPath, uint16_t srvPort);

    ~WaterServer() { gp_slider = nullptr; }

private:
    void onSliderMoveDone(const std::shared_ptr<muduo::net::TcpConnection> &sp_conn);

    void onMessage(const muduo::net::TcpConnectionPtr &conn, muduo::net::Buffer *buffer,
                   muduo::Timestamp);

    void handleCmd(const muduo::net::TcpConnectionPtr &conn, std::string cmd);

private:
    muduo::net::EventLoop *loop_;
    Sensor sensor_;
    Camera camera_;
    Slider slider_;
    muduo::net::TcpServer cmdServer_;
    muduo::ThreadPool threadPool_;
};

#endif // WATERSERVER_HPP

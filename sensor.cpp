#include "sensor.hpp"

Sensor::Sensor(muduo::net::EventLoop *loop, std::string ttyPath, std::string addr,
               uint16_t port)
    : loop_{loop}, serialPort_{ttyPath}, srvAddr_{addr, port}, client_{loop_, srvAddr_,
                                                                       "sensorClient"}
{
    client_.enableRetry();
    // client_.connect();
    serialPort_.open();
    auto fd = serialPort_.getFd();
    //        auto fd = ::open("fifo", O_RDWR);
    serialChannel_.reset(new muduo::net::Channel{loop, fd});
    serialChannel_->setReadCallback(
        std::bind(&Sensor::onMessage, this, std::placeholders::_1));
    // serialChannel_->enableReading();
}

Sensor::~Sensor()
{
    auto channel = serialChannel_.get();
    if (channel) {
        loop_->removeChannel(channel);
    }
}

void Sensor::onMessage(muduo::Timestamp)
{
    auto err = 0;
    buffer_.readFd(serialChannel_->fd(), &err);
    auto eol = buffer_.findEOL();
    if (eol) {
        std::string data = buffer_.retrieveAsString(eol - buffer_.peek() + 1);
        sendSensorData(data);
    }
}

void Sensor::sendSensorData(std::string data)
{
    if (client_.connection() && client_.connection()->connected()) {
        client_.connection()->send(data);
        LOG_INFO << data.substr(0, data.size() - 2);
    }
}

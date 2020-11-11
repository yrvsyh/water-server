#include "muduo/base/CurrentThread.h"
#include "muduo/base/Logging.h"
#include "muduo/base/Mutex.h"
#include "muduo/base/ThreadPool.h"
#include "muduo/base/Timestamp.h"
#include "muduo/net/Buffer.h"
#include "muduo/net/Channel.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/TcpClient.h"
#include "muduo/net/TcpServer.h"

#include "camera.hpp"
#include "serialport.hpp"

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <sys/signal.h>

#ifdef PI
#include <wiringPi.h>
#endif

using namespace std::placeholders;

static constexpr double COUNTER_MAX = 5333.0;

#ifdef PI
static constexpr int config_2 = 4;
static constexpr int config_3 = 5;
static constexpr int signal_A = 29;
static const std::string RTMP_URL = "rtmp://192.168.43.38:1935/live";
#else
static const std::string RTMP_URL = "rtmp://127.0.0.1:1935/live";
#endif

muduo::net::EventLoop *gp_loop;

void sliderCounter();

class Slider
{
public:
    Slider(muduo::net::EventLoop *loop, std::function<void()> cb)
        : loop_{loop}, moveDoneCallback_{cb},
          initedCallback_{[] { LOG_INFO << "init done"; }}, movedTimes_{0}, pos_{0},
          counter_{0}, lastCounter_{0}, moving_{false}, initing_{false}
    {
        timerId_ = loop_->runEvery(0.5, [this] {
            muduo::MutexLockGuard lock{mutex_};
            if (moving_ && lastCounter_ == counter_) {
                moving_ = false;
                counter_ = -1;
                if (!initing_) {
                    moveDone();
                } else {
                    initing_ = false;
                    initedCallback_();
                }
            }
            lastCounter_ = counter_;
        });
#ifdef PI
        wiringPiSetup();
        pinMode(config_2, OUTPUT);
        pinMode(config_3, OUTPUT);
        pinMode(signal_A, INPUT);
        //    pinMode(signal_B, INPUT);
        pullUpDnControl(signal_A, PUD_UP);
        wiringPiISR(signal_A, INT_EDGE_RISING, &sliderCounter); //信号A上升沿触发中断
#endif
        initHeight();
    }

    ~Slider()
    {
#ifdef PI
        digitalWrite(config_2, 0);
        digitalWrite(config_3, 0);
#endif
    }

    void count()
    {
        if (moving_ && counter_ < 0) {
            stop();
            moving_ = false;
            muduo::MutexLockGuard lock{mutex_};
            if (!initing_) {
                LOG_INFO << "moveDoneCallback";
                moveDone();
            } else {
                initing_ = false;
                initedCallback_();
            }
        }
        counter_--;
    }

    void initHeight(uint nextPos = 0)
    {
        initing_ = true;
        moving_ = true;
        pos_ = 0;
        initedCallback_ = std::bind(&Slider::move, this, nextPos);
#ifdef PI
        digitalWrite(config_2, 0);
        digitalWrite(config_3, 0);
        counter_ = 50000;
#else
        moving_ = false;
        initing_ = false;
        initedCallback_();
#endif
    }

    void stop()
    {
#ifdef PI
        digitalWrite(config_2, 1);
        digitalWrite(config_3, 0);
#endif
    }

    void move(uint pos)
    {
        moving_ = true;
        if (++movedTimes_ % 3 == 0) {
            initHeight(pos);
            return;
        }
        int dis = pos - pos_;
        LOG_INFO << "moving: " << dis;
        pos_ = pos;
        counter_ = (COUNTER_MAX / 500) * abs(dis);
#ifdef PI
        if (dis > 0) {
            digitalWrite(config_2, 1);
            digitalWrite(config_3, 1);
        } else if (dis < 0) {
            digitalWrite(config_2, 0);
            digitalWrite(config_3, 0);
        } else {
            moving_ = false;
            stop();
            moveDone();
        }
#else
//        moving_ = false;
//        moveDone();
#endif
    }

    void setMoveDoneCallback(std::function<void()> cb) { moveDoneCallback_ = cb; }

private:
    void moveDone() { loop_->runInLoop(moveDoneCallback_); }

private:
    muduo::net::EventLoop *loop_;
    std::function<void()> moveDoneCallback_;
    std::function<void()> initedCallback_;
    uint movedTimes_;
    uint pos_;
    int counter_;
    int lastCounter_;
    bool moving_;
    bool initing_;
    muduo::net::TimerId timerId_;
    muduo::MutexLock mutex_;
};

Slider *gp_slider;

void sliderCounter()
{
    if (gp_slider) {
        gp_slider->count();
    }
}

class Sensor
{
public:
    Sensor(muduo::net::EventLoop *loop, std::string ttyPath)
        : loop_{loop}, serialPort_{ttyPath}, srvAddr_{"127.0.0.1", 1234},
          client_{loop_, srvAddr_, "sensorClient"}
    {
        //        client_.enableRetry();
        //        client_.connect();
        //        serialPort_.open();
        //        auto fd = serialPort_.getFd();
        //        auto fd = ::open("fifo", O_RDWR);
        //        serialChannel_.reset(new muduo::net::Channel{loop, fd});
        //        serialChannel_->setReadCallback(std::bind(&Sensor::onMessage, this,
        //        _1));
        // serialChannel_->enableReading();
    }

    ~Sensor()
    {
        auto channel = serialChannel_.get();
        if (channel) {
            loop_->removeChannel(channel);
        }
    }

private:
    void onMessage(muduo::Timestamp)
    {
        auto err = 0;
        buffer_.readFd(serialChannel_->fd(), &err);
        auto eol = buffer_.findEOL();
        if (eol) {
            std::string msg = buffer_.retrieveAsString(eol - buffer_.peek() + 1);
            onLineMessage(msg);
        }
    }

    void onLineMessage(std::string message)
    {
        if (client_.connection() && client_.connection()->connected())
            client_.connection()->send(message);
    }

private:
    muduo::net::EventLoop *loop_;
    SerialPort serialPort_;
    muduo::net::InetAddress srvAddr_;
    muduo::net::TcpClient client_;
    muduo::net::Buffer buffer_;
    std::unique_ptr<muduo::net::Channel> serialChannel_;
};

class WaterServer
{
public:
    WaterServer(muduo::net::EventLoop *loop, std::string ttyPath, std::string camPath,
                uint16_t srvPort)
        : loop_(loop), sensor_{loop_, ttyPath}, camera_{camPath, "mjpeg", "30",
                                                        "640x480"},
          slider_{loop_, [] { LOG_INFO << "move done. no client."; }},
          cmdServer_{loop_, muduo::net::InetAddress{srvPort}, "cmdServer"},
          threadPool_{"waterThreadPool"}
    {
        gp_slider = &slider_;
        cmdServer_.setMessageCallback(
            std::bind(&WaterServer::onMessage, this, _1, _2, _3));
        cmdServer_.start();
        threadPool_.start(2);
    }

    ~WaterServer() { gp_slider = nullptr; }

private:
    void onSliderMoveDone(const std::shared_ptr<muduo::net::TcpConnection> &sp_conn)
    {
        LOG_INFO << "slider move done";
        slider_.setMoveDoneCallback([] {});
        if (sp_conn->connected()) {
            sp_conn->send("move done\n");
        } else {
            LOG_ERROR << "client closed";
        }
    }

    void onMessage(const muduo::net::TcpConnectionPtr &conn, muduo::net::Buffer *buffer,
                   muduo::Timestamp)
    {
        auto eol = buffer->findEOL();
        if (eol) {
            std::string msg = buffer->retrieveAsString(eol - buffer->peek() + 1);
            onCommand(conn, msg.substr(0, msg.length() - 1));
        }
    }

    void onCommand(const muduo::net::TcpConnectionPtr &conn, std::string cmd)
    {
        LOG_INFO << "cmd: " << cmd;
        int pos = 0;
        if (cmd == "start") {
            threadPool_.run(std::bind(&Camera::pushStream, &camera_, RTMP_URL));
        } else if (cmd == "stop") {
            slider_.stop();
        } else if (cmd == "quit") {
            threadPool_.run(std::bind(&Camera::close, &camera_));
        } else if (cmd == "cap") {
            auto img = camera_.capOneFrame("1280x720");
            auto data = std::to_string(img->size());
            data.append("\n").append(&(*img)[0], img->size());
            conn->send(data);
        } else if (1 == sscanf(cmd.c_str(), "move %d", &pos)) {
            slider_.setMoveDoneCallback(
                std::bind(&WaterServer::onSliderMoveDone, this, conn));
            slider_.move(pos);
        }
    }

private:
    muduo::net::EventLoop *loop_;
    Sensor sensor_;
    Camera camera_;
    Slider slider_;
    muduo::net::TcpServer cmdServer_;
    muduo::ThreadPool threadPool_;
};

void sighandler(int)
{
    ::printf("\r  \r");
    ::fflush(stdout);
    gp_loop->quit();
}

int main()
{
    muduo::Logger::setLogLevel(muduo::Logger::INFO);
    gp_loop = new muduo::net::EventLoop{};
    ::signal(SIGINT, sighandler);
    WaterServer water{gp_loop, "/dev/ttyUSB0", "/dev/video0", 9999};
    gp_loop->loop();
    return 0;
}

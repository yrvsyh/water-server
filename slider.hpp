#ifndef SLIDER_HPP
#define SLIDER_HPP

#include "muduo/base/Logging.h"
#include "muduo/net/EventLoop.h"

#include <memory>

class Slider
{
public:
    Slider(muduo::net::EventLoop *loop,
           std::function<void()> cb = Slider::defaultMoveDoneCallback);
    ~Slider();
    void count();
    void stop(bool reset = false);
    void move(uint pos);
    void setMoveDoneCallback(std::function<void()> cb) { moveDoneCallback_ = cb; }

private:
    void initHeight(int nextPos = -1);
    void moveDone();

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

private:
    static std::function<void()> defaultMoveDoneCallback;
    static constexpr int config_2 = 4;
    static constexpr int config_3 = 5;
    static constexpr int signal_A = 29;
    static constexpr double COUNTER_MAX = 5333.0;
};

extern Slider *gp_slider;

#endif // SLIDER_HPP

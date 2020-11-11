#include "slider.hpp"

#ifdef PI
#include <wiringPi.h>
#endif

Slider *gp_slider = nullptr;

#ifdef PI
static void sliderCounter()
{
    if (gp_slider) {
        gp_slider->count();
    }
}
#endif

Slider::Slider(muduo::net::EventLoop *loop, std::function<void()> cb)
    : loop_{loop}, moveDoneCallback_{cb}, initedCallback_{[] {
          LOG_INFO << "init done";
      }},
      movedTimes_{0}, pos_{0}, counter_{0}, lastCounter_{0}, moving_{false}, initing_{
                                                                                 false}
{
    timerId_ = loop_->runEvery(1, [this] {
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

Slider::~Slider()
{
#ifdef PI
    digitalWrite(config_2, 0);
    digitalWrite(config_3, 0);
#endif
}

void Slider::count()
{
    if (moving_ && counter_ < 0) {
        stop();
        moving_ = false;
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

void Slider::initHeight(uint nextPos)
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

void Slider::stop()
{
#ifdef PI
    digitalWrite(config_2, 1);
    digitalWrite(config_3, 0);
#endif
    // movedTimes_ = 0;
}

void Slider::move(uint pos)
{
    moving_ = true;
    if (movedTimes_++ % 50 == 0) {
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

void Slider::moveDone()
{
    loop_->queueInLoop(moveDoneCallback_);
    moveDoneCallback_ = Slider::defaultMoveDoneCallback;
}

std::function<void()> Slider::defaultMoveDoneCallback = [] {
    LOG_INFO << "move done. no client.";
};

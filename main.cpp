#include "serialport.hpp"

#include <arpa/inet.h>
#include <event2/buffer.h>
#include <event2/bufferevent.h>
#include <event2/event.h>
#include <event2/listener.h>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <queue>
#include <spdlog/spdlog.h>
#include <string>
#include <unordered_map>

#ifdef PI
#include <wiringPi.h>
#endif

#define COUNTER_MAX 5333.0
// 继电器控制口
#define config_2 4
#define config_3 5
// PWM A和B
#define signal_A 29

struct Conn
{
    Conn(bufferevent *bev) : bev(bev)
    {
    }
    bufferevent *bev;
};

struct MoveCtx
{
    MoveCtx(std::weak_ptr<Conn> conn, int pos, std::string cmd) : conn(conn), pos(pos), cmd(cmd)
    {
    }
    std::weak_ptr<Conn> conn;
    int pos;
    std::string cmd;
};

bufferevent *sensorServer = nullptr;
std::unordered_map<bufferevent *, std::shared_ptr<Conn>> clientsMap;
std::queue<MoveCtx> moveQueue;

std::function<void()> cb = [] {};

bool initing = false;
bool moving = false;
double counter = 0;
double lastCounter = 0;
int pos = 0;
int movedTimes = 0;
std::function<void()> initedCallback = [] { spdlog::info("default init callback"); };

void replyClient(std::weak_ptr<Conn> conn, int code, std::string msg, std::string cmd);

void stopSlider(bool reset)
{
#ifdef PI
    digitalWrite(config_2, 1);
    digitalWrite(config_3, 0);
#endif
    if (reset)
    {
        movedTimes = -1;
    }
    while (!moveQueue.empty())
    {
        auto &m = moveQueue.front();
        replyClient(m.conn, -1, "canceled", m.cmd);
        moveQueue.pop();
    }
}

void sliderCounter()
{
    if (moving && counter < 0)
    {
        stopSlider(false);
    }
    counter--;
}

void moveSlider(int);

void initSliderHeight(int nextPos = -1)
{
    initing = true;
    moving = true;
    pos = 0;
    if (-1 != nextPos)
    {
        initedCallback = [nextPos] { moveSlider(nextPos); };
    }
    else
    {
        initedCallback = [] { spdlog::info("default init callback"); };
    }
#ifdef PI
    digitalWrite(config_2, 0);
    digitalWrite(config_3, 0);
    counter_ = 50000;
#else
    moving = false;
    initing = false;
#endif
}

void initSlider()
{
#ifdef PI
    wiringPiSetup();
    pinMode(config_2, OUTPUT);
    pinMode(config_3, OUTPUT);
    pinMode(signal_A, INPUT);
    pullUpDnControl(signal_A, PUD_UP);
    wiringPiISR(signal_A, INT_EDGE_RISING, &sliderCounter); //信号A上升沿触发中断
#endif
    initSliderHeight();
}

void moveSlider(int p)
{
    moving = true;
    movedTimes++;
    movedTimes %= 50;
    if (movedTimes == 0)
    {
        initSliderHeight(p);
        return;
    }
    int dis = p - pos;
    spdlog::info("moving {} mm", dis);
    pos = p;
    counter = (COUNTER_MAX / 500) * abs(dis);
#ifdef PI
    if (dis > 0)
    {
        digitalWrite(config_2, 1);
        digitalWrite(config_3, 1);
    }
    else if (dis < 0)
    {
        digitalWrite(config_2, 0);
        digitalWrite(config_3, 0);
    }
    // else
    // {
    //     moving_ = false;
    //     moveDone();
    // }
#else

#endif
}

void replyClient(std::weak_ptr<Conn> conn, int code, std::string msg, std::string cmd)
{
    auto client = conn.lock();
    if (client)
    {
        auto data = fmt::format("{{\"code\": {}, \"msg\": \"{}\", \"cmd\": \"{}\"}}\n", code, msg, cmd);
        bufferevent_write(client->bev, data.c_str(), data.size());
        spdlog::info(data.substr(0, data.size() - 1));
    }
}

void checkSliderIsStoped()
{
    // spdlog::info("checking slider is stoped");
    if (moving && lastCounter == counter)
    {
        moving = false;
        counter = -1;
        if (!initing)
        {
            if (!moveQueue.empty())
            {
                replyClient(moveQueue.front().conn, 0, "done", moveQueue.front().cmd);
                moveQueue.pop();
            }
        }
        else
        {
            initing = false;
            initedCallback();
        }
    }
    if (!moving && !moveQueue.empty())
    {
        moveSlider(moveQueue.front().pos);
    }
    lastCounter = counter;
}

void slider_cb(int, short, void *)
{
    checkSliderIsStoped();
}

void doCmd(std::string cmd, std::weak_ptr<Conn> client)
{
    spdlog::info("cmd: {}", cmd);
    int pos = 0;
    if (cmd == "stop")
    {
        stopSlider(true);
    }
    else if (1 == sscanf(cmd.c_str(), "move %d", &pos))
    {
        moveQueue.push(MoveCtx(client, pos, cmd));
    }
}

void cmd_client_read_cb(bufferevent *bev, void *)
{
    auto d = evbuffer_readln(bufferevent_get_input(bev), nullptr, EVBUFFER_EOL_ANY);
    if (d)
    {
        std::string cmd(d);
        std::weak_ptr<Conn> c = clientsMap.at(bev);
        doCmd(cmd, c);
        free(d);
    }
}

void cmd_client_event_cb(bufferevent *bev, short what, void *)
{
    if (what & BEV_EVENT_EOF)
    {
        spdlog::info("client closed");
    }
    else if (what & BEV_EVENT_ERROR)
    {
        spdlog::warn("client error");
    }
    clientsMap.erase(bev);
    bufferevent_free(bev);
}

void cmd_server_listen_cb(evconnlistener *lev, int fd, sockaddr *, int, void *)
{
    spdlog::info("new client connected");
    auto bev = bufferevent_socket_new(evconnlistener_get_base(lev), fd, BEV_OPT_CLOSE_ON_FREE);
    clientsMap.insert({bev, std::make_shared<Conn>(bev)});
    bufferevent_setcb(bev, cmd_client_read_cb, nullptr, cmd_client_event_cb, nullptr);
    bufferevent_enable(bev, EV_READ);
}

void sensor_client_event_cb(bufferevent *bev, short what, void *)
{
    if (what & BEV_EVENT_CONNECTED)
    {
        spdlog::info("connected to sensor server");
        sensorServer = bev;
    }
    else
    {
        if (what & BEV_EVENT_EOF)
        {
            spdlog::warn("connection to sensor closed");
        }
        else
        {
            spdlog::warn("connect to sensor error");
        }
        sensorServer = nullptr;
        bufferevent_free(bev);
    }
}

void sensor_read_cb(bufferevent *bev, void *)
{
    auto data = evbuffer_readln(bufferevent_get_input(bev), nullptr, EVBUFFER_EOL_CRLF);
    if (data)
    {
        if (sensorServer)
        {
            bufferevent_write(sensorServer, data, strlen(data));
        }
        else
        {
            sockaddr_in sin;
            memset(&sin, 0, sizeof(sin));
            sin.sin_family = AF_INET;
            sin.sin_addr.s_addr = inet_addr("127.0.0.1");
            sin.sin_port = htons(1234);
            auto clientBev = bufferevent_socket_new(bufferevent_get_base(bev), -1, BEV_OPT_CLOSE_ON_FREE);
            bufferevent_setcb(clientBev, nullptr, nullptr, sensor_client_event_cb, nullptr);
            bufferevent_enable(clientBev, EV_READ);
            bufferevent_socket_connect(clientBev, reinterpret_cast<const sockaddr *>(&sin), sizeof(sin));
        }
        free(data);
    }
}

int main(int argc, char const *argv[])
{
    auto base = event_base_new();

    sockaddr_in sin;

    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(9999);
    auto cmdServer =
        evconnlistener_new_bind(base, cmd_server_listen_cb, nullptr, LEV_OPT_CLOSE_ON_EXEC | LEV_OPT_REUSEABLE, 5,
                                reinterpret_cast<const sockaddr *>(&sin), sizeof(sin));

    // SerialPort sensor("/dev/ttyUSB0");
    // sensor.open();
    // int fd = sensor.fd();
    int fd = open("fifo", O_RDWR);
    auto sensorBev = bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);
    bufferevent_setcb(sensorBev, sensor_read_cb, nullptr, nullptr, nullptr);
    bufferevent_enable(sensorBev, EV_READ);

    auto sliderTimer = event_new(base, 0, EV_PERSIST, slider_cb, nullptr);
    timeval timeout{1, 0};
    event_add(sliderTimer, &timeout);

    event_base_dispatch(base);
    return 0;
}

#include "serialport.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <event2/buffer.h>
#include <event2/bufferevent.h>
#include <event2/event.h>
#include <event2/listener.h>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <queue>
#include <signal.h>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>
#include <unistd.h>

#ifdef PI
#include <wiringPi.h>
#endif

#define REINIT_TIME 10
#define COUNTER_MAX 5333.0
// 继电器控制口
#define config_2 4
#define config_3 5
// PWM A
#define signal_A 29

std::string IP = "127.0.0.1";
int PORT = 1234;
int LISTEN = 9999;

struct Conn
{
    Conn(bufferevent *bev, std::string ip, int port) : bev(bev), ip(ip), port(port)
    {
    }
    bufferevent *bev;
    std::string ip;
    int port;
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
std::queue<MoveCtx> moveQueue;

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
        while (!moveQueue.empty())
        {
            auto &m = moveQueue.front();
            replyClient(m.conn, -1, "stoped", m.cmd);
            moveQueue.pop();
        }
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
        initedCallback = [nextPos] {
            spdlog::info("reinit height done");
            moveSlider(nextPos);
        };
    }
    else
    {
        initedCallback = [] { spdlog::info("default init callback"); };
    }
    counter = 50000;
#ifdef PI
    digitalWrite(config_2, 0);
    digitalWrite(config_3, 0);
#else
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
    movedTimes %= REINIT_TIME;
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
    new std::thread([p] {
        while (true)
        {
            if (moving && counter < 0)
            {
                stopSlider(false);
                break;
            }
            counter--;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
#endif
}

void replyClient(std::weak_ptr<Conn> conn, int code, std::string msg, std::string cmd)
{
    auto client = conn.lock();
    if (client)
    {
        auto data = fmt::format("{{\"code\": {}, \"msg\": \"{}\", \"cmd\": \"{}\"}}\n", code, msg, cmd);
        bufferevent_write(client->bev, data.c_str(), data.size());
        spdlog::info(data.substr(0, data.size() - 1).append(fmt::format("- {}:{}", client->ip, client->port)));
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
    int pos = 0;
    if (cmd == "stop")
    {
        moveQueue.push(MoveCtx(client, -1, cmd));
        stopSlider(true);
    }
    else if (1 == sscanf(cmd.c_str(), "move %d", &pos))
    {
        moveQueue.push(MoveCtx(client, pos, cmd));
    }
}

void cmd_client_read_cb(bufferevent *bev, void *conn)
{
    const auto &client = *reinterpret_cast<std::shared_ptr<Conn> *>(conn);
    auto d = evbuffer_readln(bufferevent_get_input(bev), nullptr, EVBUFFER_EOL_ANY);
    if (d)
    {
        std::string cmd(d);
        spdlog::info("cmd: {} - {}:{}", cmd, client->ip, client->port);
        std::weak_ptr<Conn> c = client;
        doCmd(cmd, c);
        free(d);
    }
}

void cmd_client_event_cb(bufferevent *bev, short what, void *conn)
{
    const auto &client = *reinterpret_cast<std::shared_ptr<Conn> *>(conn);
    if (what & BEV_EVENT_EOF)
    {
        spdlog::info("client closed - {}:{}", client->ip, client->port);
    }
    else if (what & BEV_EVENT_ERROR)
    {
        spdlog::warn("client error - {}:{}", client->ip, client->port);
    }
    delete &client;
    bufferevent_free(bev);
}

void cmd_server_listen_cb(evconnlistener *lev, int fd, sockaddr *addr, int socklen, void *)
{
    sockaddr_in sin;
    memcpy(&sin, addr, socklen);
    auto ip = inet_ntoa(sin.sin_addr);
    auto port = ntohs(sin.sin_port);
    spdlog::info("new client connected - {}:{}", ip, port);
    auto bev = bufferevent_socket_new(evconnlistener_get_base(lev), fd, BEV_OPT_CLOSE_ON_FREE);
    auto conn = new std::shared_ptr<Conn>(new Conn(bev, ip, port));
    bufferevent_setcb(bev, cmd_client_read_cb, nullptr, cmd_client_event_cb, conn);
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
            spdlog::debug("connect to sensor error");
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
            auto d = std::string(data).append("\n");
            bufferevent_write(sensorServer, d.c_str(), d.size());
        }
        else
        {
            sockaddr_in sin;
            memset(&sin, 0, sizeof(sin));
            sin.sin_family = AF_INET;
            sin.sin_addr.s_addr = inet_addr(IP.c_str());
            sin.sin_port = htons(PORT);
            auto clientBev = bufferevent_socket_new(bufferevent_get_base(bev), -1, BEV_OPT_CLOSE_ON_FREE);
            bufferevent_setcb(clientBev, nullptr, nullptr, sensor_client_event_cb, nullptr);
            bufferevent_enable(clientBev, EV_READ);
            bufferevent_socket_connect(clientBev, reinterpret_cast<const sockaddr *>(&sin), sizeof(sin));
        }
        free(data);
    }
}

void signal_cb(int, short, void *base)
{
    spdlog::info("exiting eventloop");
    event_base_loopexit(reinterpret_cast<event_base *>(base), nullptr);
}

int main(int argc, char const *argv[])
{
    if (argc == 4)
    {
        IP = argv[1];
        PORT = atoi(argv[2]);
        LISTEN = atoi(argv[3]);
    }

    auto base = event_base_new();

    sockaddr_in sin;

    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(LISTEN);
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
    initSlider();

    auto signalEvent = event_new(base, SIGHUP, EV_SIGNAL | EV_PERSIST, signal_cb, base);
    event_add(signalEvent, nullptr);

    event_base_dispatch(base);

    evconnlistener_free(cmdServer);
    ::close(fd);
    bufferevent_free(sensorBev);
    event_free(sliderTimer);
    event_free(signalEvent);

    return 0;
}

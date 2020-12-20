#include "muduo/base/Logging.h"
#include "muduo/net/EventLoop.h"

#include "camera.hpp"
#include "config.hpp"
#include "serialport.hpp"
#include "slider.hpp"
#include "waterserver.hpp"

#include <cstdio>
#include <getopt.h>
#include <sys/signal.h>
#include <unistd.h>

using namespace std::placeholders;

muduo::net::EventLoop *gp_loop;

void sigHandler(int)
{
    ::printf("\r  \r");
    ::fflush(stdout);
    gp_loop->quit();
}

void showUsage()
{
    puts("WaterServer [option]");
    puts("-s --sensor-path");
    puts("-a --sensor-addr");
    puts("-p --sensor-port");
    puts("-c --camera-path");
    puts("-l --listen-port");
    puts("-u --rtmp-url");
    puts("-h --help");
}

void getConfig(int argc, char *argv[])
{
    int optIndex = 0;
    static struct option longOpts[] = {{"sensor-path", required_argument, NULL, 's'},
                                       {"sensor-addr", required_argument, NULL, 'a'},
                                       {"sensor-port", required_argument, NULL, 'p'},
                                       {"camera-path", required_argument, NULL, 'c'},
                                       {"listen-port", required_argument, NULL, 'l'},
                                       {"rtmp-url", required_argument, NULL, 'u'},
                                       {"help", no_argument, NULL, 'h'},
                                       {0, 0, 0, 0}};
    while (1) {
        auto c = getopt_long(argc, argv, "s:a:p:c:l:u:h", longOpts, &optIndex);
        if (c == -1) {
            break;
        }
        switch (c) {
        case 's':
            Config::SENSOR_PATH = optarg;
            LOG_INFO << "Config::SENSOR_PATH " << Config::SENSOR_PATH;
            break;
        case 'a':
            Config::SENSOR_ADDR = optarg;
            LOG_INFO << "Config::SENSOR_ADDR " << Config::SENSOR_ADDR;
            break;
        case 'p':
            Config::SENSOR_PORT = atoi(optarg);
            LOG_INFO << "Config::SENSOR_PORT " << Config::SENSOR_PORT;
            break;
        case 'c':
            Config::CAMERA_PATH = optarg;
            LOG_INFO << Config::CAMERA_PATH;
            break;
        case 'l':
            Config::LISTEN_PORT = atoi(optarg);
            LOG_INFO << "Config::LISTEN_PORT " << Config::LISTEN_PORT;
            break;
        case 'u':
            Config::RTMP_URL = optarg;
            LOG_INFO << "Config::RTMP_URL " << Config::RTMP_URL;
            break;
        case 'h':
        default:
            showUsage();
            exit(0);
        }
    }
}

int main(int argc, char *argv[])
{
    getConfig(argc, argv);
    muduo::Logger::setLogLevel(muduo::Logger::DEBUG);
    gp_loop = new muduo::net::EventLoop{};
    ::signal(SIGINT, sigHandler);
    WaterServer water{gp_loop,
                      Config::SENSOR_PATH,
                      Config::SENSOR_ADDR,
                      Config::SENSOR_PORT,
                      Config::CAMERA_PATH,
                      Config::LISTEN_PORT};
    gp_loop->loop();
    return 0;
}

#include "waterserver.hpp"
#include "config.hpp"

WaterServer::WaterServer(muduo::net::EventLoop *loop, std::string ttyPath,
                         std::string sensorAddr, uint16_t sensorPort, std::string camPath,
                         uint16_t srvPort)
    : loop_(loop), sensor_{loop_, ttyPath, sensorAddr, sensorPort}, camera_{camPath,
                                                                            "mjpeg", "30",
                                                                            "640x480"},
      slider_{loop_}, cmdServer_{loop_, muduo::net::InetAddress{srvPort}, "cmdServer"},
      threadPool_{"waterThreadPool"}
{
    gp_slider = &slider_;
    cmdServer_.setMessageCallback(std::bind(&WaterServer::onMessage, this, _1, _2, _3));
    cmdServer_.start();
    threadPool_.start(2);
}

void WaterServer::onSliderMoveDone(
    muduo::net::TcpConnection *p_conn)
{
    LOG_INFO << "slider move done";
    if (p_conn && p_conn->connected()) {
        p_conn->send("move done\n");
    } else {
        LOG_ERROR << "client closed";
    }
}

void WaterServer::onMessage(const muduo::net::TcpConnectionPtr &conn,
                            muduo::net::Buffer *buffer, muduo::Timestamp)
{
    auto eol = buffer->findEOL();
    if (eol) {
        std::string cmd = buffer->retrieveAsString(eol - buffer->peek() + 1);
        handleCmd(conn, cmd.substr(0, cmd.size() - 1));
    }
}

void WaterServer::handleCmd(const muduo::net::TcpConnectionPtr &conn, std::string cmd)
{
    LOG_INFO << "cmd: " << cmd;
    int pos = 0;
    if (cmd == "start") {
        threadPool_.run(std::bind(&Camera::pushStream, &camera_, Config::RTMP_URL));
    } else if (cmd == "stop") {
        slider_.stop();
    } else if (cmd == "quit") {
        threadPool_.run(std::bind(&Camera::close, &camera_));
    } else if (cmd == "cap") {
        auto img = camera_.capOneFrame("3264x2448", "15");
        auto data = std::to_string(img->size());
	LOG_INFO << "caped " << data << " bytes";
        data.append("\n").append(&(*img)[0], img->size());
        conn->send(data);
    } else if (1 == sscanf(cmd.c_str(), "move %d", &pos)) {
        slider_.setMoveDoneCallback(
            std::bind(&WaterServer::onSliderMoveDone, this, conn.get()));
        slider_.move(pos);
    }
}

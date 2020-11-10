#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <memory>
#include <string>
#include <vector>

class Camera
{
public:
    Camera(std::string device_path, std::string input_format, std::string framerate,
           std::string video_size);
    ~Camera();
    void open();
    void close();
    std::shared_ptr<std::vector<char>> capOneFrame(std::string video_size = "",
                                                   std::string framerate = "");
    void changeVideoOption(std::string input_format, std::string framerate,
                           std::string video_size);
    void pushStream(std::string url);
    void stopPush();

private:
    Camera(const Camera &c);
    Camera &operator=(const Camera &);
    struct M;
    M *m;
};

#endif // CAMERA_HPP

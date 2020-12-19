#include "serialport.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

struct SerialPort::M
{
    std::string path;
    OpenOptions open_options;
    bool is_open;
    int tty_fd;
};

const SerialPort::OpenOptions SerialPort::defaultOptions = {
    true,                   // bool autoOpen;
    SerialPort::BR115200,   // BaudRate baudRate;
    SerialPort::DataBits8,  // DataBits dataBits;
    SerialPort::StopBits1,  // StopBits stopBits;
    SerialPort::ParityNone, // Parity parity;
    false,                  // input xon
    false,                  // input xoff
    false,                  // input xany
    0,                      // c_cc vmin
    50,                     // c_cc vtime
};

SerialPort::SerialPort(const std::string path, const OpenOptions &options)
{
    m = new M{};
    m->path = path;
    m->open_options = options;
    m->is_open = false;
    if (options.autoOpen)
    {
        m->is_open = open(m->path, m->open_options);
    }
}

SerialPort::~SerialPort()
{
    close();
    delete m;
}

bool SerialPort::open()
{
    if (!m->is_open)
    {
        m->is_open = open(m->path, m->open_options);
    }
    return m->is_open;
}

bool SerialPort::open(const std::string path, const OpenOptions &options)
{
    if (m->path != path)
    {
        m->path = path;
    }
    if (m->open_options != options)
    {
        m->open_options = options;
    }

    m->tty_fd = ::open(m->path.c_str(), O_RDWR | O_NOCTTY);
    if (m->tty_fd < 0)
    {
        return false;
    }

    struct termios tios;
    termiosOptions(tios, options);
    tcsetattr(m->tty_fd, TCSANOW, &tios);
    tcflush(m->tty_fd, TCIOFLUSH);

    return true;
}

int SerialPort::fd()
{
    return m->tty_fd;
}

int SerialPort::read(void *data, int length)
{
    return ::read(m->tty_fd, data, length);
}

int SerialPort::write(const void *data, int length)
{
    return ::write(m->tty_fd, data, length);
}

void SerialPort::close()
{
    ::close(m->tty_fd);
    m->is_open = false;
}

void SerialPort::termiosOptions(termios &tios, const OpenOptions &options)
{
    tcgetattr(m->tty_fd, &tios);

    cfmakeraw(&tios);
    tios.c_cflag &= ~(CSIZE | CRTSCTS);
    tios.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR);
    tios.c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE | ICANON | ECHO | ECHOE | ISIG);
    tios.c_oflag &= ~(OPOST | ONLCR);

    cfsetispeed(&tios, options.baudRate);
    cfsetospeed(&tios, options.baudRate);

    tios.c_iflag |= (options.xon ? IXON : 0) | (options.xoff ? IXOFF : 0) | (options.xany ? IXANY : 0);

    // data bits
    int databits[] = {CS5, CS6, CS7, CS8};
    tios.c_cflag &= ~0x30;
    tios.c_cflag |= databits[options.dataBits];

    // stop bits
    if (options.stopBits == StopBits2)
    {
        tios.c_cflag |= CSTOPB;
    }
    else
    {
        tios.c_cflag &= ~CSTOPB;
    }

    // parity
    if (options.parity == ParityNone)
    {
        tios.c_cflag &= ~PARENB;
    }
    else
    {
        tios.c_cflag |= PARENB;

        if (options.parity == PariteMark)
        {
            tios.c_cflag |= PARMRK;
        }
        else
        {
            tios.c_cflag &= ~PARMRK;
        }

        if (options.parity == ParityOdd)
        {
            tios.c_cflag |= PARODD;
        }
        else
        {
            tios.c_cflag &= ~PARODD;
        }
    }

    tios.c_cc[VMIN] = options.vmin;
    tios.c_cc[VTIME] = options.vtime;
}

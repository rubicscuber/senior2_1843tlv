#include "serial_port.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

speed_t baudToSpeed(int baud)
{
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return B0;
    }
}

} // namespace

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& device, int baud)
{
    close();
    const speed_t speed = baudToSpeed(baud);
    if (speed == B0) {
        std::fprintf(stderr, "Error: unsupported baud rate %d\n", baud);
        return false;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "Error: %s could not be opened! (%s)\n",
                     device.c_str(), std::strerror(errno));
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        std::fprintf(stderr, "Error: tcgetattr failed for %s (%s)\n",
                     device.c_str(), std::strerror(errno));
        close();
        return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD); // no modem control, enable receiver
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~PARENB;          // no parity
    tty.c_cflag &= ~CRTSCTS;         // no hardware flow control
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::fprintf(stderr, "Error: tcsetattr failed for %s (%s)\n",
                     device.c_str(), std::strerror(errno));
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    std::printf("%s opened.\n", device.c_str());
    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::bytesAvailable() const
{
    if (fd_ < 0)
        return 0;
    int count = 0;
    if (ioctl(fd_, FIONREAD, &count) < 0)
        return 0;
    return count;
}

int SerialPort::readBytes(uint8_t* buf, size_t maxLen)
{
    if (fd_ < 0 || maxLen == 0)
        return 0;
    const ssize_t n = ::read(fd_, buf, maxLen);
    return (n > 0) ? static_cast<int>(n) : 0;
}

bool SerialPort::writeLine(const std::string& line)
{
    if (fd_ < 0)
        return false;
    std::string data = line + "\n";
    size_t written = 0;
    while (written < data.size()) {
        const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            return false;
        }
        written += static_cast<size_t>(n);
    }
    tcdrain(fd_);
    return true;
}

std::string SerialPort::readLine(int timeoutMs)
{
    std::string line;
    if (fd_ < 0)
        return line;
    while (true) {
        char c;
        const ssize_t n = ::read(fd_, &c, 1);
        if (n == 1) {
            if (c == '\n')
                return line;
            if (c != '\r')
                line.push_back(c);
            continue;
        }
        // no byte available: wait for one within the remaining timeout
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(fd_, &readSet);
        timeval tv{};
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        const int ready = select(fd_ + 1, &readSet, nullptr, nullptr, &tv);
        if (ready <= 0)
            return line; // timeout or error: return what we have
    }
}

bool loadCfg(SerialPort& cfgPort, const std::vector<std::string>& cfgLines)
{
    std::printf("Sending cfg file to device...\n");
    for (const auto& line : cfgLines) {
        // skip empty (whitespace-only) lines and comments
        const size_t firstNonSpace = line.find_first_not_of(" \t");
        if (firstNonSpace == std::string::npos)
            continue;
        if (line[firstNonSpace] == '%')
            continue;

        if (!cfgPort.writeLine(line)) {
            std::fprintf(stderr, "Error: failed to write to cfg port.\n");
            return false;
        }
        std::printf("%s\n", line.c_str());

        for (int k = 0; k < 3; k++) {
            const std::string response = cfgPort.readLine(1000);
            if (response == "Done") {
                std::printf("%s\n", response.c_str());
                break;
            }
            if (response.find("not recognized as a CLI command") != std::string::npos
                || response.find("Error") != std::string::npos) {
                std::printf("%s\n", response.c_str());
                return false;
            }
        }
        usleep(20000); // 20 ms between commands, as in loadCfg.m
    }
    return true;
}

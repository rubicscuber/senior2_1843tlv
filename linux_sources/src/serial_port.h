// POSIX (termios) serial port access.
// Replaces the MATLAB serial objects used by initCfgPort.m, initDataPort.m,
// loadCfg.m and readUARTtoBuffer.m.
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <cstdint>
#include <string>
#include <vector>

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Open `device` (e.g. "/dev/ttyACM0") in raw 8N1 mode at `baud`
    // (115200 for the cfg port, 921600 for the data port).
    bool open(const std::string& device, int baud);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Number of bytes waiting in the input buffer (0 on error).
    int bytesAvailable() const;

    // Non-blocking read of up to maxLen bytes; returns bytes read (0 if none).
    int readBytes(uint8_t* buf, size_t maxLen);

    // Write a CLI command line, appending '\n'.
    bool writeLine(const std::string& line);

    // Read one LF-terminated line, waiting up to timeoutMs. Returns the line
    // without the terminator; empty string on timeout.
    std::string readLine(int timeoutMs);

private:
    int fd_ = -1;
};

// Send every non-comment cfg line to the device's CLI port and wait for its
// "Done" acknowledgement (port of loadCfg.m). Returns false on a CLI error.
bool loadCfg(SerialPort& cfgPort, const std::vector<std::string>& cfgLines);

#endif // SERIAL_PORT_H

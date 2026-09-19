// POSIX (termios) serial port access.
// Replaces the MATLAB serial objects used by initCfgPort.m, initDataPort.m,
// loadCfg.m and readUARTtoBuffer.m.
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <atomic>
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

    // Wait up to timeoutMs for input, then return bytesAvailable(). open()
    // flushes stale input, so use this (not bytesAvailable()) to decide
    // whether a device is already streaming.
    int bytesAvailableWithin(int timeoutMs) const;

    // Non-blocking read of up to maxLen bytes; returns bytes read (0 if none)
    // or -1 (errno set) once the port has hung up or failed, e.g. after the
    // USB device was unplugged. The port is then unusable: give up on it.
    int readBytes(uint8_t* buf, size_t maxLen);

    // Write a CLI command line, appending '\n'.
    bool writeLine(const std::string& line);

    // Read one LF-terminated line, waiting up to timeoutMs in total. Returns
    // the line without the terminator; what was received so far on timeout.
    std::string readLine(int timeoutMs);

private:
    int fd_ = -1;
};

// Send every non-comment cfg line to the device's CLI port and wait for its
// "Done" acknowledgement (port of loadCfg.m). Returns false on a CLI error,
// or when *keepRunning turns false (Ctrl-C) while the cfg is being sent.
bool loadCfg(SerialPort& cfgPort, const std::vector<std::string>& cfgLines,
             const std::atomic<bool>* keepRunning = nullptr);

#endif // SERIAL_PORT_H

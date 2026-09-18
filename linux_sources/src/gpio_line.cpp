#include "gpio_line.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/gpio.h>

#ifndef GPIO_V2_GET_LINE_IOCTL
#error "Linux GPIO uAPI v2 headers (linux-libc-dev from kernel >= 5.10) are required"
#endif

#ifndef ENOTSUPP
#define ENOTSUPP 524 // kernel-internal errno that can leak to user space
#endif

namespace {

void reportChipOpenError(const std::string& path, int err)
{
    if (err == EACCES || err == EPERM) {
        std::fprintf(stderr,
                     "Error: permission denied opening %s.\n"
                     "       Add your user to the gpio group (sudo usermod -aG gpio $USER, then log in\n"
                     "       again) or run install_boot_service.sh, which sets this up.\n",
                     path.c_str());
    } else if (err == ENOENT) {
        std::fprintf(stderr,
                     "Error: %s does not exist: no GPIO character device on this system.\n"
                     "       Use --gpiochip <path> for a different chip, or --sim-gpio / --self-speed\n"
                     "       to run without GPIO hardware.\n",
                     path.c_str());
    } else {
        std::fprintf(stderr, "Error: could not open %s: %s\n", path.c_str(), std::strerror(err));
    }
}

void reportRequestError(const GpioLineId& id, int err)
{
    if (err == EBUSY) {
        std::fprintf(stderr,
                     "Error: %s (%s line %u) is already in use by another process "
                     "(see gpioinfo for the consumer).\n",
                     id.name.c_str(), id.chipPath.c_str(), id.offset);
    } else if (err == EINVAL) {
        std::fprintf(stderr,
                     "Error: the kernel rejected the request for %s line %u (EINVAL): "
                     "offset out of range or unsupported flags.\n",
                     id.chipPath.c_str(), id.offset);
    } else {
        std::fprintf(stderr, "Error: requesting %s line %u failed: %s\n",
                     id.chipPath.c_str(), id.offset, std::strerror(err));
    }
}

// Open the chip, submit the line request, close the chip. Returns the line fd
// or -1 (errOut set; chip-open errors are reported here, request errors are
// left to the caller so it can retry with different attributes).
int requestLine(const GpioLineId& id, gpio_v2_line_request& req, int& errOut)
{
    const int chipFd = ::open(id.chipPath.c_str(), O_RDONLY | O_CLOEXEC);
    if (chipFd < 0) {
        errOut = errno;
        reportChipOpenError(id.chipPath, errOut);
        return -1;
    }
    const int rc = ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &req);
    errOut = (rc == 0) ? 0 : errno;
    ::close(chipFd);
    return (rc == 0) ? req.fd : -1;
}

} // namespace

bool resolveBcmGpio(int bcm, const std::string& chipOverride, GpioLineId& out)
{
    if (bcm < 0) {
        std::fprintf(stderr, "Error: invalid GPIO number %d\n", bcm);
        return false;
    }
    const std::string wanted = "GPIO" + std::to_string(bcm);
    if (!chipOverride.empty()) {
        out = {chipOverride, static_cast<uint32_t>(bcm), wanted};
        return true;
    }

    bool sawChip = false;
    int permErr = 0;
    for (int c = 0; c < 32; c++) {
        const std::string path = "/dev/gpiochip" + std::to_string(c);
        const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) {
            if (errno == EACCES || errno == EPERM)
                permErr = errno;
            continue;
        }
        sawChip = true;
        gpiochip_info ci{};
        if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &ci) == 0) {
            for (uint32_t i = 0; i < ci.lines; i++) {
                gpio_v2_line_info li{};
                li.offset = i;
                if (ioctl(fd, GPIO_V2_GET_LINEINFO_IOCTL, &li) != 0)
                    continue;
                if (std::strncmp(li.name, wanted.c_str(), GPIO_MAX_NAME_SIZE) == 0) {
                    ::close(fd);
                    out = {path, i, wanted};
                    return true;
                }
            }
        }
        ::close(fd);
    }

    if (!sawChip) {
        reportChipOpenError("/dev/gpiochip0", permErr ? permErr : ENOENT);
        return false;
    }
    std::fprintf(stderr,
                 "Note: no GPIO line named %s found; using /dev/gpiochip0 line %d "
                 "(BCM numbering, correct on Raspberry Pi 3/4).\n",
                 wanted.c_str(), bcm);
    out = {"/dev/gpiochip0", static_cast<uint32_t>(bcm), wanted};
    return true;
}

// ---- GpioInput ----

GpioInput::~GpioInput()
{
    close();
}

bool GpioInput::open(const GpioLineId& id, const Config& cfg)
{
    close();

    gpio_v2_line_request req{}; // every uAPI struct must be zero-initialised (padding is checked)
    req.offsets[0] = id.offset;
    req.num_lines = 1;
    std::snprintf(req.consumer, sizeof(req.consumer), "%s", cfg.consumer.c_str());
    req.event_buffer_size = cfg.eventBufferSize;

    uint64_t flags = GPIO_V2_LINE_FLAG_INPUT;
    flags |= (cfg.edge == Edge::Rising) ? GPIO_V2_LINE_FLAG_EDGE_RISING
                                        : GPIO_V2_LINE_FLAG_EDGE_FALLING;
    switch (cfg.bias) {
    case Bias::PullUp:   flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP; break;
    case Bias::PullDown: flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN; break;
    case Bias::None:     flags |= GPIO_V2_LINE_FLAG_BIAS_DISABLED; break;
    }
    req.config.flags = flags;

    if (cfg.debounceUs > 0) {
        req.config.num_attrs = 1;
        req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
        req.config.attrs[0].attr.debounce_period_us = cfg.debounceUs;
        req.config.attrs[0].mask = 1;
    }

    int err = 0;
    int fd = requestLine(id, req, err);
    if (fd < 0 && err != 0 && cfg.debounceUs > 0
        && (err == EINVAL || err == ENOTSUPP || err == EOPNOTSUPP)) {
        std::fprintf(stderr,
                     "Note: the kernel does not support debounce on %s line %u; "
                     "relying on the software glitch filter.\n",
                     id.chipPath.c_str(), id.offset);
        req.config.num_attrs = 0;
        std::memset(req.config.attrs, 0, sizeof(req.config.attrs));
        req.fd = 0;
        fd = requestLine(id, req, err);
    }
    if (fd < 0) {
        if (err != EACCES && err != EPERM && err != ENOENT) // chip-open errors already reported
            reportRequestError(id, err);
        return false;
    }

    fd_ = fd;
    debounce_ = (req.config.num_attrs == 1);

    // the request fd is created blocking; prefer O_NONBLOCK, else poll() first
    const int fl = fcntl(fd_, F_GETFL);
    usePoll_ = (fl < 0) || (fcntl(fd_, F_SETFL, fl | O_NONBLOCK) < 0);
    return true;
}

void GpioInput::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    debounce_ = false;
    usePoll_ = false;
}

int GpioInput::readEvents(std::vector<Event>& out)
{
    if (fd_ < 0)
        return -1;
    gpio_v2_line_event buf[64];
    int total = 0;
    for (;;) {
        if (usePoll_) {
            pollfd p{};
            p.fd = fd_;
            p.events = POLLIN;
            if (::poll(&p, 1, 0) <= 0 || !(p.revents & POLLIN))
                break;
        }
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (n == 0)
            break;
        const size_t count = static_cast<size_t>(n) / sizeof(gpio_v2_line_event);
        for (size_t i = 0; i < count; i++) {
            out.push_back({buf[i].timestamp_ns, buf[i].line_seqno,
                           buf[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE});
        }
        total += static_cast<int>(count);
        if (count < sizeof(buf) / sizeof(buf[0]))
            break; // queue drained
    }
    return total;
}

int GpioInput::readValue() const
{
    if (fd_ < 0)
        return -1;
    gpio_v2_line_values v{};
    v.mask = 1;
    if (ioctl(fd_, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) != 0)
        return -1;
    return static_cast<int>(v.bits & 1u);
}

// ---- GpioOutput ----

GpioOutput::~GpioOutput()
{
    close();
}

bool GpioOutput::open(const GpioLineId& id, bool activeLow, const std::string& consumer)
{
    close();

    gpio_v2_line_request req{};
    req.offsets[0] = id.offset;
    req.num_lines = 1;
    std::snprintf(req.consumer, sizeof(req.consumer), "%s", consumer.c_str());
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT | (activeLow ? GPIO_V2_LINE_FLAG_ACTIVE_LOW : 0);
    // start with the LED off (also clears an LED left on by a crashed run)
    req.config.num_attrs = 1;
    req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    req.config.attrs[0].attr.values = 0;
    req.config.attrs[0].mask = 1;

    int err = 0;
    const int fd = requestLine(id, req, err);
    if (fd < 0) {
        if (err != EACCES && err != EPERM && err != ENOENT)
            reportRequestError(id, err);
        return false;
    }
    fd_ = fd;
    return true;
}

bool GpioOutput::set(bool on)
{
    if (fd_ < 0)
        return false;
    gpio_v2_line_values v{};
    v.mask = 1;
    v.bits = on ? 1 : 0;
    return ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) == 0;
}

void GpioOutput::close()
{
    if (fd_ >= 0) {
        set(false);
        ::close(fd_);
        fd_ = -1;
    }
}

const char* gpioEdgeName(GpioInput::Edge edge)
{
    return edge == GpioInput::Edge::Rising ? "rising" : "falling";
}

const char* gpioBiasName(GpioInput::Bias bias)
{
    switch (bias) {
    case GpioInput::Bias::PullUp:   return "pull-up";
    case GpioInput::Bias::PullDown: return "pull-down";
    case GpioInput::Bias::None:     return "no bias";
    }
    return "?";
}

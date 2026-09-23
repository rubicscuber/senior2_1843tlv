#include "alert_sound.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include "mono_clock.h"

extern char** environ;

namespace {

constexpr uint64_t NS_PER_S = 1000000000ull;
constexpr uint64_t KILL_GRACE_NS = NS_PER_S; // SIGTERM -> SIGKILL after this

std::vector<std::string> splitWords(const std::string& s)
{
    std::vector<std::string> words;
    std::istringstream iss(s);
    std::string w;
    while (iss >> w)
        words.push_back(w);
    return words;
}

bool executableFound(const std::string& cmd)
{
    if (cmd.find('/') != std::string::npos)
        return access(cmd.c_str(), X_OK) == 0;
    const char* env = std::getenv("PATH");
    const std::string path = env ? env : "/usr/local/bin:/usr/bin:/bin";
    size_t start = 0;
    while (start <= path.size()) {
        size_t end = path.find(':', start);
        if (end == std::string::npos)
            end = path.size();
        std::string dir = path.substr(start, end - start);
        if (dir.empty())
            dir = ".";
        if (access((dir + "/" + cmd).c_str(), X_OK) == 0)
            return true;
        start = end + 1;
    }
    return false;
}

} // namespace

AlertSound::AlertSound(const AlertSoundParams& p, uint64_t epochNs)
    : p_(p), argv_(splitWords(p.player)), epochNs_(epochNs)
{
    argv_.push_back(p_.file);
}

AlertSound::~AlertSound()
{
    if (pid_ > 0) {
        ::kill(pid_, SIGTERM);
        reap(true);
    }
}

bool AlertSound::check(std::string& err) const
{
    if (p_.file.empty()) {
        err = "no audio file given";
        return false;
    }
    if (access(p_.file.c_str(), R_OK) != 0) {
        err = "cannot read audio file " + p_.file + ": " + std::strerror(errno);
        return false;
    }
    if (p_.simulate)
        return true;
    if (argv_.size() < 2) {
        err = "empty player command";
        return false;
    }
    if (!executableFound(argv_[0])) {
        err = "player '" + argv_[0] + "' not found in PATH (install alsa-utils for aplay, "
              "or set --sound-player)";
        return false;
    }
    return true;
}

void AlertSound::print(const char* what, uint64_t nowNs) const
{
    const double t = (nowNs >= epochNs_) ? (nowNs - epochNs_) * 1e-9 : 0.0;
    std::printf("[SOUND] %-5s %s  t=%.3f s\n", what, p_.file.c_str(), t);
}

bool AlertSound::spawn()
{
    std::vector<char*> argv;
    for (auto& a : argv_)
        argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);

    // the player must never read our terminal or write over the report;
    // its stderr stays visible so ALSA errors ("audio open error") show up
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&fa, STDOUT_FILENO, "/dev/null", O_WRONLY, 0);
    pid_t pid = -1;
    const int rc = posix_spawnp(&pid, argv[0], &fa, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    if (rc != 0) {
        std::fprintf(stderr, "Warning: could not start '%s': %s\n", argv_[0].c_str(), std::strerror(rc));
        return false;
    }
    pid_ = pid;
    stopping_ = false;
    wantStart_ = false;
    starts_++;
    return true;
}

bool AlertSound::reap(bool block)
{
    if (pid_ <= 0)
        return true;
    int status = 0;
    const pid_t r = ::waitpid(pid_, &status, block ? 0 : WNOHANG);
    if (r == 0)
        return false; // still running
    if (r < 0 && errno == EINTR)
        return block ? reap(true) : false;
    // exited (or vanished): if it failed on its own, say so once per start
    if (r > 0 && !stopping_ && WIFEXITED(status) && WEXITSTATUS(status) != 0)
        std::fprintf(stderr, "Warning: '%s' exited with status %d (wrong audio device or file?)\n",
                     argv_[0].c_str(), WEXITSTATUS(status));
    pid_ = -1;
    stopping_ = false;
    return true;
}

bool AlertSound::playOnce(double timeoutS)
{
    if (p_.simulate) {
        print("check", monotonicNowNs());
        return true;
    }
    if (pid_ > 0)
        stop(monotonicNowNs());
    if (!spawn())
        return false;
    const uint64_t deadline = monotonicNowNs() + static_cast<uint64_t>(timeoutS * 1e9);
    int status = 0;
    while (true) {
        const pid_t r = ::waitpid(pid_, &status, WNOHANG);
        if (r == pid_)
            break;
        if (r < 0 && errno != EINTR) {
            pid_ = -1;
            return false;
        }
        if (monotonicNowNs() >= deadline) {
            ::kill(pid_, SIGTERM);
            reap(true);
            std::fprintf(stderr, "Warning: sound check: player still running after %.0f s, stopped it.\n",
                         timeoutS);
            return true; // it did play; the file is just long
        }
        usleep(20000);
    }
    pid_ = -1;
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

void AlertSound::onApproachAlert(bool active, uint64_t nowNs)
{
    if (active == active_)
        return;
    active_ = active;
    if (p_.simulate) {
        print(active ? "start" : "stop", nowNs);
        if (active)
            starts_++;
        return;
    }
    if (active) {
        wantStart_ = true;
        if (pid_ <= 0)
            spawn();
        // else the previous stop is still in flight: poll() starts us afterwards
    } else {
        stop(nowNs);
    }
}

void AlertSound::stop(uint64_t nowNs)
{
    wantStart_ = false;
    if (p_.simulate || pid_ <= 0 || stopping_)
        return;
    ::kill(pid_, SIGTERM);
    stopping_ = true;
    killAtNs_ = nowNs + KILL_GRACE_NS;
    reap(false); // usually gone at once
}

void AlertSound::poll(uint64_t nowNs)
{
    if (p_.simulate)
        return;
    if (pid_ > 0) {
        if (!reap(false)) {
            if (stopping_ && nowNs >= killAtNs_) {
                ::kill(pid_, SIGKILL);
                killAtNs_ = nowNs + KILL_GRACE_NS;
            }
            return;
        }
    }
    // no child running now: (re)start while the alert is active
    if (active_ && (wantStart_ || p_.repeat))
        spawn();
}

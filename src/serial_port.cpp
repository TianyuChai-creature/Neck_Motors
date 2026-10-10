#include "serial_port.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>

namespace f32c {
namespace {

std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm local {};
    localtime_r(&t, &local);
    std::ostringstream out;
    out << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0')
        << std::setw(3) << ms.count();
    return out.str();
}

void log_bytes(std::ofstream& log, const char* kind, const uint8_t* data, size_t n) {
    if (!log.is_open()) return;
    log << timestamp() << ' ' << kind << ' ' << n << "B";
    for (size_t i = 0; i < n; ++i) {
        log << ' ' << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(data[i]);
    }
    log << std::dec << std::setfill(' ') << '\n';
    log.flush();
}

void log_text(std::ofstream& log, const char* kind, const std::string& text) {
    if (!log.is_open()) return;
    log << timestamp() << ' ' << kind << ' ' << text << '\n';
    log.flush();
}

speed_t baud_flag(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200;
    }
}

}  // namespace

bool SerialPort::open(const std::string& device, int baud) {
    close();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tio {};
    if (tcgetattr(fd_, &tio) != 0) {
        close();
        return false;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_flag(baud));
    cfsetospeed(&tio, baud_flag(baud));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        close();
        return false;
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    // CDC: assert DTR
#ifdef TIOCM_DTR
    int m = TIOCM_DTR;
    ioctl(fd_, TIOCMBIS, &m);
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flush_input();
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    stop_log();
}

bool SerialPort::write_all(const uint8_t* data, size_t n) {
    if (fd_ < 0) return false;
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::write(fd_, data + off, n - off);
        if (w < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            log_text(log_, "TX_ERROR", std::strerror(errno));
            return false;
        }
        off += static_cast<size_t>(w);
    }
    log_bytes(log_, "TX", data, n);
    return true;
}

std::vector<uint8_t> SerialPort::read_for(int timeout_ms) {
    std::vector<uint8_t> out;
    if (fd_ < 0) return out;
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    auto idle_until = deadline;
    bool got = false;
    uint8_t buf[128];
    while (clock::now() < deadline) {
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            out.insert(out.end(), buf, buf + n);
            log_bytes(log_, "RX", buf, static_cast<size_t>(n));
            got = true;
            idle_until = clock::now() + std::chrono::milliseconds(3);
        } else if (got && clock::now() >= idle_until) {
            break;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return out;
}

void SerialPort::flush_input() {
    if (fd_ >= 0) {
        tcflush(fd_, TCIFLUSH);
        log_text(log_, "FLUSH", "input");
    }
}

bool SerialPort::start_log(const std::string& path) {
    stop_log();
    log_.open(path, std::ios::out | std::ios::app);
    if (!log_.is_open()) return false;
    log_text(log_, "LOG", "started");
    return true;
}

void SerialPort::stop_log() {
    if (log_.is_open()) {
        log_text(log_, "LOG", "stopped");
        log_.close();
    }
}

}  // namespace f32c

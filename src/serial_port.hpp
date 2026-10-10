#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace f32c {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud = 115200);
    void close();
    bool is_open() const { return fd_ >= 0; }

    bool write_all(const uint8_t* data, size_t n);
    bool write_all(const std::vector<uint8_t>& v) { return write_all(v.data(), v.size()); }
    std::vector<uint8_t> read_for(int timeout_ms);
    void flush_input();
    bool start_log(const std::string& path);
    void stop_log();

private:
    int fd_ = -1;
    std::ofstream log_;
};

}  // namespace f32c

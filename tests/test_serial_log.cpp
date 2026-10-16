#include "serial_port.hpp"

#include <pty.h>
#include <unistd.h>

#include "check.hpp"
#include <cstdio>
#include <fstream>
#include <string>

int main() {
    int master = -1;
    int slave = -1;
    char slave_name[128] = {};
    CHECK(openpty(&master, &slave, slave_name, nullptr, nullptr) == 0);
    close(slave);

    const std::string log_path = "/tmp/f32c_serial_log_test.log";
    unlink(log_path.c_str());

    f32c::SerialPort port;
    CHECK(port.open(slave_name, 115200));
    CHECK(port.start_log(log_path));

    const uint8_t tx[] = {0x7A, 0x01, 0x06, 0x7D, 0x7B};
    CHECK(port.write_all(tx, sizeof(tx)));
    uint8_t observed[sizeof(tx)] = {};
    CHECK(read(master, observed, sizeof(observed)) == static_cast<ssize_t>(sizeof(observed)));

    const uint8_t rx[] = {0x7A, 0x01, 0x02, 0x00, 0x00, 0x09, 0xF0, 0x80, 0x7B};
    CHECK(write(master, rx, sizeof(rx)) == static_cast<ssize_t>(sizeof(rx)));
    const auto received = port.read_for(20);
    CHECK(received.size() == sizeof(rx));
    port.stop_log();
    port.close();
    close(master);

    std::ifstream log(log_path);
    const std::string text((std::istreambuf_iterator<char>(log)), std::istreambuf_iterator<char>());
    CHECK(text.find(" TX 5B 7A 01 06 7D 7B") != std::string::npos);
    CHECK(text.find(" RX 9B 7A 01 02 00 00 09 F0 80 7B") != std::string::npos);
    std::remove(log_path.c_str());
    std::puts("serial log passed");
}

#pragma once

#include "serial_port.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace f32c {

class Motor {
public:
    Motor(SerialPort& port, uint8_t id);

    uint8_t id() const { return id_; }

    bool enable();
    bool disable();
    bool set_pi(uint16_t speed_kp, uint16_t speed_ki, uint16_t pos_kp, uint16_t pos_ki);
    bool setup_position(int travel_rpm, uint16_t accel, double initial_deg);
    bool set_deg(double deg);
    bool read_deg(double* deg, int timeout_ms = 50);
    bool set_zero();
    bool save();

private:
    bool send(const std::vector<uint8_t>& pkt);
    bool query_mech(double* deg, int timeout_ms);

    SerialPort& port_;
    uint8_t id_;
};

}  // namespace f32c

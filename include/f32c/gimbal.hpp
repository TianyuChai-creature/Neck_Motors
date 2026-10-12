#pragma once

#include <cstdint>
#include <string>

namespace f32c {

class Gimbal2Dof {
public:
    struct Config {
        // Empty means scan serial devices and select the one answering both IDs.
        std::string device;
        int baud = 115200;
        uint8_t id_yaw = 1;
        uint8_t id_pitch = 2;
        int travel_rpm = 80;
        uint16_t accel = 10;
        // Current-load gains; yaw +/-60 passed at pitch=0, other postures need validation.
        uint16_t yaw_speed_kp = 50;
        uint16_t yaw_speed_ki = 16;
        uint16_t yaw_pos_kp = 63;
        uint16_t yaw_pos_ki = 0;
        uint16_t pitch_speed_kp = 30;
        uint16_t pitch_speed_ki = 8;
        uint16_t pitch_pos_kp = 60;
        uint16_t pitch_pos_ki = 0;
        Config() = default;
    };

    struct State {
        bool ok = false;
        double yaw_deg = 0;
        double pitch_deg = 0;
    };

    Gimbal2Dof();
    explicit Gimbal2Dof(Config cfg);
    ~Gimbal2Dof();

    Gimbal2Dof(const Gimbal2Dof&) = delete;
    Gimbal2Dof& operator=(const Gimbal2Dof&) = delete;

    bool open();
    void close();
    // Releases the port even on failure. True means no pending cleanup, not motor ACK.
    bool close_checked();

    bool ping();
    bool enable();
    // Enable with explicit initial targets; no smoothing. Existing enable() holds feedback.
    bool enable_at(double yaw_deg, double pitch_deg);
    bool disable();
    // axis 0: yaw, 1: pitch. Update volatile PI while holding; no save or mode change.
    bool set_pi(unsigned axis, unsigned speed_kp, unsigned speed_ki, unsigned pos_kp, unsigned pos_ki);
    // Immediate direct-mode setpoint. Callers may generate smoothed intermediate targets.
    bool set_deg(double yaw_deg, double pitch_deg);
    bool set_yaw_deg(double yaw_deg);
    bool set_pitch_deg(double pitch_deg);
    bool read(State* out);
    bool set_zero();
    bool start_log(const std::string& path);
    void stop_log();
    const std::string& last_error() const;

private:
    bool enable_impl(const State* initial_target);
    struct Impl;
    Impl* impl_;
};

}  // namespace f32c

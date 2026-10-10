#include "f32c/gimbal.hpp"

#include "motor.hpp"
#include "serial_port.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <system_error>
#include <utility>
#include <vector>

namespace f32c {
namespace {

bool starts_with(const std::string& value, const char* prefix) {
    return value.rfind(prefix, 0) == 0;
}

std::vector<std::string> serial_candidates() {
    std::vector<std::string> out;
    auto add_dir = [&out](const char* dir, bool all_entries) {
        std::error_code ec;
        for (std::filesystem::directory_iterator it(dir, ec), end; it != end && !ec; it.increment(ec)) {
            const auto name = it->path().filename().string();
            if (all_entries || starts_with(name, "ttyACM") || starts_with(name, "ttyUSB")) {
                out.push_back(it->path().string());
            }
        }
    };
    add_dir("/dev/serial/by-id", true);
    add_dir("/dev", false);
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace

struct Gimbal2Dof::Impl {
    Config cfg;
    SerialPort port;
    Motor yaw;
    Motor pitch;
    bool armed = false;
    std::string error;

    explicit Impl(Config c)
        : cfg(std::move(c)), yaw(port, cfg.id_yaw), pitch(port, cfg.id_pitch) {}

    bool fail(std::string message) {
        error = std::move(message);
        return false;
    }
};

Gimbal2Dof::Gimbal2Dof() : Gimbal2Dof(Config()) {}

Gimbal2Dof::Gimbal2Dof(Config cfg) : impl_(new Impl(std::move(cfg))) {}

Gimbal2Dof::~Gimbal2Dof() {
    if (impl_) {
        if (impl_->armed) disable();
        delete impl_;
        impl_ = nullptr;
    }
}

bool Gimbal2Dof::open() {
    const auto& c = impl_->cfg;
    if (impl_->armed) return impl_->fail("disable motors before reopening");
    if (!c.id_yaw || !c.id_pitch || c.id_yaw == c.id_pitch)
        return impl_->fail("motor IDs must be nonzero and distinct");
    if (c.baud != 9600 && c.baud != 57600 && c.baud != 115200)
        return impl_->fail("unsupported baud rate");
    if (c.travel_rpm < 0 || c.travel_rpm > 1000)
        return impl_->fail("travel_rpm must be between 0 and 1000");
    if (c.yaw_speed_kp > 256 || c.yaw_speed_ki > 256 || c.yaw_pos_kp > 256 || c.yaw_pos_ki > 256 ||
        c.pitch_speed_kp > 256 || c.pitch_speed_ki > 256 || c.pitch_pos_kp > 256 || c.pitch_pos_ki > 256)
        return impl_->fail("PI gains must be between 0 and 256");

    if (!c.device.empty()) {
        if (!impl_->port.open(c.device, c.baud))
            return impl_->fail("cannot open serial device: " + c.device);
        impl_->error.clear();
        return true;
    }
    for (const auto& device : serial_candidates()) {
        if (!impl_->port.open(device, c.baud)) continue;
        if (ping()) {
            impl_->cfg.device = device;
            impl_->error.clear();
            return true;
        }
        impl_->port.close();
    }
    return impl_->fail("no serial device responded for both motor IDs");
}

void Gimbal2Dof::close() {
    if (impl_->armed) disable();
    impl_->port.close();
}

bool Gimbal2Dof::ping() {
    double y = 0, p = 0;
    if (!impl_->yaw.read_deg(&y)) return impl_->fail("yaw read failed");
    if (!impl_->pitch.read_deg(&p)) return impl_->fail("pitch read failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::enable() {
    if (impl_->armed) { impl_->error.clear(); return true; }
    auto& y = impl_->yaw;
    auto& p = impl_->pitch;
    const auto& c = impl_->cfg;
    const bool ok = y.enable() && p.enable() &&
                    y.setup_position(c.travel_rpm, c.accel, c.yaw_speed_kp, c.yaw_speed_ki,
                                     c.yaw_pos_kp, c.yaw_pos_ki) &&
                    p.setup_position(c.travel_rpm, c.accel, c.pitch_speed_kp, c.pitch_speed_ki,
                                     c.pitch_pos_kp, c.pitch_pos_ki);
    if (!ok) {
        y.disable();
        p.disable();
        impl_->armed = false;
        return impl_->fail("motor enable or position setup failed");
    }
    double ydeg = 0, pdeg = 0;
    if (!y.read_deg(&ydeg) || !p.read_deg(&pdeg) || !y.set_deg(ydeg) || !p.set_deg(pdeg)) {
        y.disable();
        p.disable();
        impl_->armed = false;
        return impl_->fail("could not hold the current angle after enable");
    }
    impl_->armed = true;
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::disable() {
    const bool a = impl_->yaw.disable();
    const bool b = impl_->pitch.disable();
    impl_->armed = false;
    if (!a || !b) return impl_->fail("motor disable write failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_deg(double yaw_deg, double pitch_deg) {
    if (!impl_->armed) return impl_->fail("motors are disabled");
    if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg))
        return impl_->fail("target angles must be finite");
    if (!impl_->yaw.set_deg(yaw_deg)) return impl_->fail("yaw target write failed");
    if (!impl_->pitch.set_deg(pitch_deg)) return impl_->fail("pitch target write failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_pi(unsigned axis, unsigned speed_kp, unsigned speed_ki, unsigned pos_kp, unsigned pos_ki) {
    if (axis > 1) return impl_->fail("PI axis must be 0 (yaw) or 1 (pitch)");
    if (speed_kp > 256 || speed_ki > 256 || pos_kp > 256 || pos_ki > 256)
        return impl_->fail("PI gains must be between 0 and 256");
    if (!impl_->armed) return impl_->fail("motors are disabled");
    auto& motor = axis == 0 ? impl_->yaw : impl_->pitch;
    if (!motor.set_pi(speed_kp, speed_ki, pos_kp, pos_ki))
        return impl_->fail("runtime PI write failed");
    auto& c = impl_->cfg;
    if (axis == 0) {
        c.yaw_speed_kp = speed_kp; c.yaw_speed_ki = speed_ki;
        c.yaw_pos_kp = pos_kp; c.yaw_pos_ki = pos_ki;
    } else {
        c.pitch_speed_kp = speed_kp; c.pitch_speed_ki = speed_ki;
        c.pitch_pos_kp = pos_kp; c.pitch_pos_ki = pos_ki;
    }
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_yaw_deg(double yaw_deg) {
    if (!impl_->armed) return impl_->fail("motors are disabled");
    if (!std::isfinite(yaw_deg)) return impl_->fail("target angle must be finite");
    if (!impl_->yaw.set_deg(yaw_deg)) return impl_->fail("yaw target write failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_pitch_deg(double pitch_deg) {
    if (!impl_->armed) return impl_->fail("motors are disabled");
    if (!std::isfinite(pitch_deg)) return impl_->fail("target angle must be finite");
    if (!impl_->pitch.set_deg(pitch_deg)) return impl_->fail("pitch target write failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::read(State* out) {
    if (!out) return impl_->fail("output state is null");
    State s;
    if (!impl_->yaw.read_deg(&s.yaw_deg)) {
        *out = s;
        return impl_->fail("yaw read failed");
    }
    if (!impl_->pitch.read_deg(&s.pitch_deg)) {
        *out = s;
        return impl_->fail("pitch read failed");
    }
    s.ok = true;
    *out = s;
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_zero() {
    if (!impl_->armed) return impl_->fail("motors are disabled");
    if (!impl_->yaw.set_zero()) return impl_->fail("yaw zero write failed");
    if (!impl_->pitch.set_zero()) return impl_->fail("pitch zero write failed");
    if (!impl_->yaw.save()) return impl_->fail("yaw parameter save failed");
    if (!impl_->pitch.save()) return impl_->fail("pitch parameter save failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::start_log(const std::string& path) {
    if (path.empty() || !impl_->port.start_log(path)) return impl_->fail("cannot open serial log");
    impl_->error.clear();
    return true;
}

void Gimbal2Dof::stop_log() { impl_->port.stop_log(); }

const std::string& Gimbal2Dof::last_error() const { return impl_->error; }

}  // namespace f32c

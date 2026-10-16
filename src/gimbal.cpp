#include "f32c/gimbal.hpp"

#include "motor.hpp"
#include "serial_port.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
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
    bool cleanup_pending = false;
    std::string error;

    explicit Impl(Config c)
        : cfg(std::move(c)), yaw(port, cfg.id_yaw), pitch(port, cfg.id_pitch) {}

    bool fail(std::string message) {
        error = std::move(message);
        return false;
    }

    bool require_enabled() {
        return armed || fail(cleanup_pending ? "disable request failed; hardware state unknown"
                                            : "motors are not enabled by this instance");
    }
};

Gimbal2Dof::Gimbal2Dof() : Gimbal2Dof(Config()) {}

Gimbal2Dof::Gimbal2Dof(Config cfg) : impl_(new Impl(std::move(cfg))) {}

Gimbal2Dof::~Gimbal2Dof() {
    if (impl_) {
        if (impl_->port.is_open()) close();
        delete impl_;
        impl_ = nullptr;
    }
}

bool Gimbal2Dof::open() {
    const auto& c = impl_->cfg;
    if (impl_->armed || impl_->cleanup_pending)
        return impl_->fail("disable requests must succeed before reopening; hardware state unconfirmed");
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
    if (!close_checked()) std::fprintf(stderr, "F32C close failed: %s\n", last_error().c_str());
}

bool Gimbal2Dof::close_checked() {
    if (!impl_->port.is_open()) {
        if (impl_->cleanup_pending) return impl_->fail("serial port closed after failed disable; hardware state unknown");
        return true;
    }
    const bool ok = !(impl_->armed || impl_->cleanup_pending) || disable();
    impl_->port.close();
    return ok;
}

bool Gimbal2Dof::ping() {
    double y = 0, p = 0;
    if (!impl_->yaw.read_deg(&y)) return impl_->fail("yaw read failed");
    if (!impl_->pitch.read_deg(&p)) return impl_->fail("pitch read failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::enable() {
    return enable_impl(nullptr);
}

bool Gimbal2Dof::enable_at(double yaw_deg, double pitch_deg) {
    if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg))
        return impl_->fail("target angles must be finite");
    if (impl_->armed) return set_deg(yaw_deg, pitch_deg);
    State target;
    target.yaw_deg = yaw_deg;
    target.pitch_deg = pitch_deg;
    return enable_impl(&target);
}

bool Gimbal2Dof::enable_impl(const State* initial_target) {
    if (impl_->armed) { impl_->error.clear(); return true; }
    auto& y = impl_->yaw;
    auto& p = impl_->pitch;
    const auto& c = impl_->cfg;
    double ydeg = 0, pdeg = 0;
    if (initial_target) {
        ydeg = initial_target->yaw_deg;
        pdeg = initial_target->pitch_deg;
    } else if (!y.read_deg(&ydeg) || !p.read_deg(&pdeg)) {
        return impl_->fail("could not read the current angle before enable");
    }
    // Program the complete gain sets without torque before starting either axis.
    if (!disable()) return false;
    impl_->cleanup_pending = true;
    const bool ok = y.set_pi(c.yaw_speed_kp, c.yaw_speed_ki, c.yaw_pos_kp, c.yaw_pos_ki) &&
                    p.set_pi(c.pitch_speed_kp, c.pitch_speed_ki, c.pitch_pos_kp, c.pitch_pos_ki) &&
                    y.enable() && y.setup_position(c.travel_rpm, c.accel, ydeg) &&
                    p.enable() && p.setup_position(c.travel_rpm, c.accel, pdeg);
    if (!ok) {
        const bool cleaned = disable();
        return impl_->fail("motor enable or position setup failed; " +
                          (cleaned ? std::string("disable requests sent, hardware state unconfirmed")
                                   : impl_->error));
    }
    impl_->armed = true;
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::disable() {
    const bool a = impl_->yaw.disable();
    const bool b = impl_->pitch.disable();
    impl_->armed = false;
    impl_->cleanup_pending = !a || !b;
    if (!a || !b)
        return impl_->fail(std::string("disable request write failed for ") +
                           (!a && !b ? "yaw and pitch" : !a ? "yaw" : "pitch") +
                           "; hardware state unknown");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_deg(double yaw_deg, double pitch_deg) {
    if (!impl_->require_enabled()) return false;
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
    if (!impl_->require_enabled()) return false;
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
    if (!impl_->require_enabled()) return false;
    if (!std::isfinite(yaw_deg)) return impl_->fail("target angle must be finite");
    if (!impl_->yaw.set_deg(yaw_deg)) return impl_->fail("yaw target write failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_pitch_deg(double pitch_deg) {
    if (!impl_->require_enabled()) return false;
    if (!std::isfinite(pitch_deg)) return impl_->fail("target angle must be finite");
    if (!impl_->pitch.set_deg(pitch_deg)) return impl_->fail("pitch target write failed");
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::read(State* out) {
    return read_timeout(out, 100);
}

bool Gimbal2Dof::read_timeout(State* out, int timeout_ms) {
    if (!out) return impl_->fail("output state is null");
    if (timeout_ms <= 0 || timeout_ms > 1000) return impl_->fail("read timeout must be within 1..1000 ms");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    auto remaining = [&] { return std::max(0, static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count())); };
    State s;
    if (!impl_->yaw.read_deg(&s.yaw_deg, std::min(50, remaining()))) {
        *out = s;
        return impl_->fail("yaw read failed");
    }
    if (!impl_->pitch.read_deg(&s.pitch_deg, std::min(50, remaining()))) {
        *out = s;
        return impl_->fail("pitch read failed");
    }
    s.ok = true;
    *out = s;
    impl_->error.clear();
    return true;
}

bool Gimbal2Dof::set_zero() {
    if (!impl_->require_enabled()) return false;
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

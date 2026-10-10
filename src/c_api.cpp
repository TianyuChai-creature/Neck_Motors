#include "f32c/c_api.h"
#include "f32c/gimbal.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

struct f32c_gimbal {
    f32c::Gimbal2Dof g;
    explicit f32c_gimbal(f32c::Gimbal2Dof::Config cfg) : g(std::move(cfg)) {}
};

namespace {
thread_local std::string open_error;

bool valid_config(const f32c_config& c) {
    if (!c.yaw_id || !c.pitch_id || c.yaw_id > 255 || c.pitch_id > 255 || c.yaw_id == c.pitch_id) {
        open_error = "motor IDs must be nonzero, distinct, and at most 255";
        return false;
    }
    if (c.accel > UINT16_MAX || c.yaw_speed_kp > 256 || c.yaw_speed_ki > 256 ||
        c.yaw_pos_kp > 256 || c.yaw_pos_ki > 256 || c.pitch_speed_kp > 256 ||
        c.pitch_speed_ki > 256 || c.pitch_pos_kp > 256 || c.pitch_pos_ki > 256) {
        open_error = "acceleration or PI gain is out of range";
        return false;
    }
    return true;
}
}  // namespace

void f32c_config_defaults(f32c_config* out) {
    if (!out) return;
    const f32c::Gimbal2Dof::Config c;
    *out = {nullptr, c.baud, c.id_yaw, c.id_pitch, c.travel_rpm, c.accel,
            c.yaw_speed_kp, c.yaw_speed_ki, c.yaw_pos_kp, c.yaw_pos_ki,
            c.pitch_speed_kp, c.pitch_speed_ki, c.pitch_pos_kp, c.pitch_pos_ki};
}

f32c_gimbal* f32c_open_config(const f32c_config* config) {
    if (!config) {
        open_error = "config is null";
        return nullptr;
    }
    if (!valid_config(*config)) return nullptr;
    f32c::Gimbal2Dof::Config c;
    if (config->device) c.device = config->device;
    c.baud = config->baud;
    c.id_yaw = static_cast<uint8_t>(config->yaw_id);
    c.id_pitch = static_cast<uint8_t>(config->pitch_id);
    c.travel_rpm = config->travel_rpm;
    c.accel = static_cast<uint16_t>(config->accel);
    c.yaw_speed_kp = static_cast<uint16_t>(config->yaw_speed_kp);
    c.yaw_speed_ki = static_cast<uint16_t>(config->yaw_speed_ki);
    c.yaw_pos_kp = static_cast<uint16_t>(config->yaw_pos_kp);
    c.yaw_pos_ki = static_cast<uint16_t>(config->yaw_pos_ki);
    c.pitch_speed_kp = static_cast<uint16_t>(config->pitch_speed_kp);
    c.pitch_speed_ki = static_cast<uint16_t>(config->pitch_speed_ki);
    c.pitch_pos_kp = static_cast<uint16_t>(config->pitch_pos_kp);
    c.pitch_pos_ki = static_cast<uint16_t>(config->pitch_pos_ki);

    auto h = std::make_unique<f32c_gimbal>(std::move(c));
    if (!h->g.open() || !h->g.ping()) {
        open_error = h->g.last_error();
        return nullptr;
    }
    open_error.clear();
    return h.release();
}

f32c_gimbal* f32c_open(const char* device, unsigned yaw_id, unsigned pitch_id) {
    f32c_config c;
    f32c_config_defaults(&c);
    c.device = device;
    c.yaw_id = yaw_id;
    c.pitch_id = pitch_id;
    return f32c_open_config(&c);
}

void f32c_close(f32c_gimbal* g) {
    if (!g) return;
    g->g.disable();
    g->g.close();
    delete g;
}

const char* f32c_last_error(const f32c_gimbal* g) {
    return g ? g->g.last_error().c_str() : open_error.c_str();
}

int f32c_ping(f32c_gimbal* g) { return g && g->g.ping() ? 1 : 0; }
int f32c_enable(f32c_gimbal* g) { return g && g->g.enable() ? 1 : 0; }
int f32c_disable(f32c_gimbal* g) { return g && g->g.disable() ? 1 : 0; }
int f32c_set_pi(f32c_gimbal* g, unsigned axis, unsigned speed_kp, unsigned speed_ki,
               unsigned pos_kp, unsigned pos_ki) {
    return g && g->g.set_pi(axis, speed_kp, speed_ki, pos_kp, pos_ki) ? 1 : 0;
}

int f32c_set_deg(f32c_gimbal* g, double yaw_deg, double pitch_deg) {
    return g && g->g.set_deg(yaw_deg, pitch_deg) ? 1 : 0;
}
int f32c_set_yaw_deg(f32c_gimbal* g, double yaw_deg) {
    return g && g->g.set_yaw_deg(yaw_deg) ? 1 : 0;
}
int f32c_set_pitch_deg(f32c_gimbal* g, double pitch_deg) {
    return g && g->g.set_pitch_deg(pitch_deg) ? 1 : 0;
}

int f32c_read(f32c_gimbal* g, double* yaw_deg, double* pitch_deg) {
    if (!g || !yaw_deg || !pitch_deg) return 0;
    f32c::Gimbal2Dof::State s;
    if (!g->g.read(&s)) return 0;
    *yaw_deg = s.yaw_deg;
    *pitch_deg = s.pitch_deg;
    return 1;
}

int f32c_set_zero(f32c_gimbal* g) { return g && g->g.set_zero() ? 1 : 0; }

int f32c_start_log(f32c_gimbal* g, const char* path) {
    return g && path && g->g.start_log(path) ? 1 : 0;
}

void f32c_stop_log(f32c_gimbal* g) {
    if (g) g->g.stop_log();
}

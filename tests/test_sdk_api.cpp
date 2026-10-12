#include "f32c/c_api.h"
#include "f32c/gimbal.hpp"

#include <cassert>
#include <limits>
#include <string>

int main() {
    f32c_config c;
    f32c_config_defaults(&c);
    assert(c.yaw_id == 1 && c.pitch_id == 2);
    assert(c.yaw_speed_kp == 50 && c.yaw_speed_ki == 16);
    assert(c.yaw_pos_kp == 63 && c.yaw_pos_ki == 0);
    assert(c.pitch_speed_kp == 30 && c.pitch_speed_ki == 8);
    assert(c.pitch_pos_kp == 60 && c.pitch_pos_ki == 0);

    c.pitch_id = 256;
    assert(f32c_open_config(&c) == nullptr);
    assert(std::string(f32c_last_error(nullptr)).find("ID") != std::string::npos);

    f32c::Gimbal2Dof::Config config;
    config.id_pitch = 1;
    f32c::Gimbal2Dof g(config);
    assert(!g.open());
    assert(g.last_error().find("distinct") != std::string::npos);
    assert(!g.set_deg(0, 0));
    assert(g.last_error() == "motors are not enabled by this instance");
    assert(!g.enable_at(std::numeric_limits<double>::infinity(), 0));
    assert(g.last_error() == "target angles must be finite");
    assert(!f32c_enable_at(nullptr, 0, 0));
    assert(!g.set_pi(0, 257, 0, 40, 0));
    assert(g.last_error().find("PI gains") != std::string::npos);
    assert(!g.set_pi(2, 40, 0, 40, 0));
    assert(g.last_error().find("PI axis") != std::string::npos);
    assert(!g.set_pi(0, 40, 0, 40, 0));
    assert(g.last_error() == "motors are not enabled by this instance");
    assert(!f32c_set_pi(nullptr, 0, 40, 0, 40, 0));
    assert(g.close_checked());
    assert(f32c_close_checked(nullptr));
}

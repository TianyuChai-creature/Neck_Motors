#include "f32c/c_api.h"
#include "f32c/gimbal.hpp"

#include "check.hpp"
#include <limits>
#include <string>

int main() {
    f32c_config c;
    f32c_config_defaults(&c);
    CHECK(c.yaw_id == 1 && c.pitch_id == 2);
    CHECK(c.travel_rpm == 15 && c.accel == 2);
    CHECK(c.yaw_speed_kp == 40 && c.yaw_speed_ki == 4);
    CHECK(c.yaw_pos_kp == 42 && c.yaw_pos_ki == 0);
    CHECK(c.pitch_speed_kp == 10 && c.pitch_speed_ki == 4);
    CHECK(c.pitch_pos_kp == 40 && c.pitch_pos_ki == 0);

    c.pitch_id = 256;
    CHECK(f32c_open_config(&c) == nullptr);
    CHECK(std::string(f32c_last_error(nullptr)).find("ID") != std::string::npos);

    f32c::Gimbal2Dof::Config config;
    config.id_pitch = 1;
    f32c::Gimbal2Dof g(config);
    CHECK(!g.open());
    CHECK(g.last_error().find("distinct") != std::string::npos);
    CHECK(!g.set_deg(0, 0));
    CHECK(g.last_error() == "motors are not enabled by this instance");
    CHECK(!g.enable_at(std::numeric_limits<double>::infinity(), 0));
    CHECK(g.last_error() == "target angles must be finite");
    CHECK(!f32c_enable_at(nullptr, 0, 0));
    CHECK(!g.set_pi(0, 257, 0, 40, 0));
    CHECK(g.last_error().find("PI gains") != std::string::npos);
    CHECK(!g.set_pi(2, 40, 0, 40, 0));
    CHECK(g.last_error().find("PI axis") != std::string::npos);
    CHECK(!g.set_pi(0, 40, 0, 40, 0));
    CHECK(g.last_error() == "motors are not enabled by this instance");
    CHECK(!f32c_set_pi(nullptr, 0, 40, 0, 40, 0));
    CHECK(g.close_checked());
    CHECK(f32c_close_checked(nullptr));
}

#include "f32c/protocol.hpp"

#include <cstdio>
#include <string>
#include <vector>

using f32c::find_frame;

static std::string hex(const std::vector<uint8_t>& v) {
    std::string s;
    char buf[8];
    for (size_t i = 0; i < v.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%s%02X", i ? " " : "", v[i]);
        s += buf;
    }
    return s;
}

static int g_fail = 0;

static void expect_eq(const char* name, const std::vector<uint8_t>& got, const std::vector<uint8_t>& want) {
    if (got == want) {
        std::printf("ok  %s  %s\n", name, hex(got).c_str());
        return;
    }
    std::printf("FAIL %s\n  got  %s\n  want %s\n", name, hex(got).c_str(), hex(want).c_str());
    ++g_fail;
}

int main() {
    using namespace f32c;

    expect_eq("enable id2", cmd_enable(2), {0x7A, 0x02, 0x06, 0x7E, 0x7B});
    expect_eq("disable id2", cmd_disable(2), {0x7A, 0x02, 0x05, 0x7D, 0x7B});
    expect_eq("save id2", cmd_save(2), {0x7A, 0x02, 0x08, 0x70, 0x7B});
    expect_eq("save id1", cmd_save(1), {0x7A, 0x01, 0x08, 0x73, 0x7B});
    expect_eq("set_zero id2", cmd_set_zero(2), {0x7A, 0x02, 0x0A, 0x72, 0x7B});
    expect_eq("mode single-turn T id2", cmd_mode_single_turn_t(2),
              {0x7A, 0x02, 0x00, 0x00, 0x02, 0x7A, 0x7B});
    expect_eq("mode single-turn direct id2", cmd_mode_single_turn_direct(2),
              {0x7A, 0x02, 0x00, 0x00, 0x04, 0x7C, 0x7B});
    expect_eq("speed kp=10 id2", cmd_speed_kp(2, 10), {0x7A, 0x02, 0x0F, 0x00, 0x0A, 0x7D, 0x7B});
    expect_eq("speed ki=10 id2", cmd_speed_ki(2, 10), {0x7A, 0x02, 0x10, 0x00, 0x0A, 0x62, 0x7B});
    expect_eq("pos kp=10 id2", cmd_pos_kp(2, 10), {0x7A, 0x02, 0x11, 0x00, 0x0A, 0x63, 0x7B});
    expect_eq("pos ki=10 id2", cmd_pos_ki(2, 10), {0x7A, 0x02, 0x12, 0x00, 0x0A, 0x60, 0x7B});
    expect_eq("speed 100rpm id2", cmd_speed_rpm(2, 100), {0x7A, 0x02, 0x01, 0x00, 0x64, 0x1D, 0x7B});
    expect_eq("accel 100 id2", cmd_accel(2, 100), {0x7A, 0x02, 0x07, 0x00, 0x64, 0x1B, 0x7B});
    expect_eq("single-turn 150deg id2", cmd_single_turn_deg(2, 150.0),
              {0x7A, 0x02, 0x03, 0x05, 0xDC, 0xA2, 0x7B});
    expect_eq("single-turn 0deg id2", cmd_single_turn_deg(2, 0.0),
              {0x7A, 0x02, 0x03, 0x00, 0x00, 0x7B, 0x7B});
    expect_eq("single-turn large finite angle id2", cmd_single_turn_deg(2, 3600000000.0),
              {0x7A, 0x02, 0x03, 0x00, 0x00, 0x7B, 0x7B});
    expect_eq("feedback mech id2", cmd_feedback_mech(2), {0x7A, 0x02, 0x0E, 0x02, 0x74, 0x7B});
    expect_eq("feedback mech id1", cmd_feedback_mech(1), {0x7A, 0x01, 0x0E, 0x02, 0x77, 0x7B});

    // 7B in payload must not be treated as tail: 9-byte window + BCC
    const uint8_t mech[] = {0x7A, 0x01, 0x02, 0x00, 0x00, 0x05, 0xC8, 0xB4, 0x7B};
    auto fr = find_frame(mech, sizeof(mech));
    if (!fr || fr->addr != 1 || fr->func != 2 || !feedback_i32(*fr) || *feedback_i32(*fr) != 1480) {
        std::printf("FAIL parse mech 148.0\n");
        ++g_fail;
    } else {
        std::printf("ok  parse mech 148.0\n");
    }

    const uint8_t bad[] = {0x7A, 0x02, 0x02, 0x00, 0x00, 0x01, 0x06, 0x2C, 0x7B};  // BCC wrong
    if (find_frame(bad, sizeof(bad))) {
        std::printf("FAIL rejected corrupt BCC\n");
        ++g_fail;
    } else {
        std::printf("ok  rejected corrupt BCC\n");
    }

    if (g_fail) {
        std::printf("%d failed\n", g_fail);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}

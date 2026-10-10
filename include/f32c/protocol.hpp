#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace f32c {

constexpr uint8_t kHead = 0x7A;
constexpr uint8_t kTail = 0x7B;

constexpr uint16_t kSingleTurnTrapezoid = 2;
constexpr uint16_t kSingleTurnDirect = 4;

enum class Feedback : uint8_t { MechAngleX10 = 2 };

inline uint8_t bcc_of(const uint8_t* p, size_t n) {
    uint8_t x = 0;
    for (size_t i = 0; i < n; ++i) x ^= p[i];
    return x;
}

inline void put_be16(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(static_cast<uint8_t>(x >> 8));
    v.push_back(static_cast<uint8_t>(x));
}

inline int32_t be32s(const uint8_t* p) {
    return static_cast<int32_t>((uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
                                (uint32_t(p[2]) << 8) | uint32_t(p[3]));
}

inline std::vector<uint8_t> pack(uint8_t addr, uint8_t func, const uint8_t* data, size_t n) {
    std::vector<uint8_t> f;
    f.reserve(n + 5);
    f.push_back(kHead);
    f.push_back(addr);
    f.push_back(func);
    if (n && data) f.insert(f.end(), data, data + n);
    f.push_back(bcc_of(f.data(), f.size()));
    f.push_back(kTail);
    return f;
}

inline std::vector<uint8_t> cmd_enable(uint8_t addr) { return pack(addr, 0x06, nullptr, 0); }
inline std::vector<uint8_t> cmd_disable(uint8_t addr) { return pack(addr, 0x05, nullptr, 0); }
inline std::vector<uint8_t> cmd_save(uint8_t addr) { return pack(addr, 0x08, nullptr, 0); }
inline std::vector<uint8_t> cmd_set_zero(uint8_t addr) { return pack(addr, 0x0A, nullptr, 0); }

inline std::vector<uint8_t> cmd_mode_single_turn_t(uint8_t addr) {
    std::vector<uint8_t> d;
    put_be16(d, kSingleTurnTrapezoid);
    return pack(addr, 0x00, d.data(), d.size());
}

inline std::vector<uint8_t> cmd_mode_single_turn_direct(uint8_t addr) {
    std::vector<uint8_t> d;
    put_be16(d, kSingleTurnDirect);
    return pack(addr, 0x00, d.data(), d.size());
}

inline std::vector<uint8_t> cmd_pid_u16(uint8_t addr, uint8_t func, uint16_t value) {
    std::vector<uint8_t> d;
    put_be16(d, value);
    return pack(addr, func, d.data(), d.size());
}

inline std::vector<uint8_t> cmd_speed_kp(uint8_t addr, uint16_t v) { return cmd_pid_u16(addr, 0x0F, v); }
inline std::vector<uint8_t> cmd_speed_ki(uint8_t addr, uint16_t v) { return cmd_pid_u16(addr, 0x10, v); }
inline std::vector<uint8_t> cmd_pos_kp(uint8_t addr, uint16_t v) { return cmd_pid_u16(addr, 0x11, v); }
inline std::vector<uint8_t> cmd_pos_ki(uint8_t addr, uint16_t v) { return cmd_pid_u16(addr, 0x12, v); }

inline std::vector<uint8_t> cmd_speed_rpm(uint8_t addr, int16_t rpm) {
    std::vector<uint8_t> d;
    put_be16(d, static_cast<uint16_t>(rpm));
    return pack(addr, 0x01, d.data(), d.size());
}

inline std::vector<uint8_t> cmd_accel(uint8_t addr, uint16_t rev_per_s2) {
    std::vector<uint8_t> d;
    put_be16(d, rev_per_s2);
    return pack(addr, 0x07, d.data(), d.size());
}

inline std::vector<uint8_t> cmd_single_turn_deg(uint8_t addr, double deg) {
    int v = static_cast<int>(std::fmod(deg, 360.0) * 10.0);
    v %= 3600;
    if (v < 0) v += 3600;
    std::vector<uint8_t> d;
    put_be16(d, static_cast<uint16_t>(v));
    return pack(addr, 0x03, d.data(), d.size());
}

inline std::vector<uint8_t> cmd_feedback_mech(uint8_t addr) {
    const uint8_t t = static_cast<uint8_t>(Feedback::MechAngleX10);
    return pack(addr, 0x0E, &t, 1);
}

struct Frame {
    uint8_t addr = 0;
    uint8_t func = 0;
    std::vector<uint8_t> data;
};

inline std::optional<Frame> parse_at(const uint8_t* buf, size_t n, size_t off, size_t* consumed) {
    static constexpr size_t kLens[] = {5, 6, 7, 9};
    if (off >= n || buf[off] != kHead) return std::nullopt;
    for (size_t len : kLens) {
        if (off + len > n) continue;
        if (buf[off + len - 1] != kTail) continue;
        if (bcc_of(buf + off, len - 2) != buf[off + len - 2]) continue;
        Frame f;
        f.addr = buf[off + 1];
        f.func = buf[off + 2];
        if (len > 5) f.data.assign(buf + off + 3, buf + off + len - 2);
        if (consumed) *consumed = off + len;
        return f;
    }
    return std::nullopt;
}

inline std::optional<Frame> find_frame(const uint8_t* buf, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        auto f = parse_at(buf, n, i, nullptr);
        if (f) return f;
    }
    return std::nullopt;
}

inline std::optional<int32_t> feedback_i32(const Frame& f) {
    if (f.data.size() != 4) return std::nullopt;
    return be32s(f.data.data());
}

}  // namespace f32c

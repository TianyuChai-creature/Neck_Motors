#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace f32c::camera {

class ZedMini {
public:
    enum class View { Left, Right, SideBySide };
    enum class Setting {
        Brightness,
        Contrast,
        Saturation,
        Sharpness,
        Gain,
        Exposure,
        WhiteBalanceTemperature,
        LedStatus,
    };

    struct ImageView {
        // BGRA8 CPU memory; valid until the same view is retrieved again or close() is called.
        const uint8_t* data = nullptr;
        size_t size_bytes = 0;
        size_t stride_bytes = 0;
        int width = 0;
        int height = 0;
        int channels = 0;
        uint64_t timestamp_ns = 0;
    };

    ZedMini();
    ~ZedMini();

    ZedMini(const ZedMini&) = delete;
    ZedMini& operator=(const ZedMini&) = delete;

    // Opens the first ZED camera, or a specific serial number when non-zero.
    bool open(unsigned int serial_number = 0);
    void close();
    bool is_open() const;

    bool grab();
    bool retrieve(View view, ImageView* out);

    bool set_setting(Setting setting, int value);
    bool get_setting(Setting setting, int* value);

    unsigned int serial_number() const;
    const std::string& last_error() const;

private:
    struct Impl;
    Impl* impl_;
};

}  // namespace f32c::camera

#include "f32c/camera/zed_mini.hpp"

#include <sl/Camera.hpp>

#include <array>

namespace f32c::camera {
namespace {

std::string error_text(sl::ERROR_CODE code) {
    return sl::toString(code).c_str();
}

bool to_zed_setting(ZedMini::Setting setting, sl::VIDEO_SETTINGS* out) {
    if (!out) return false;
    switch (setting) {
        case ZedMini::Setting::Brightness: *out = sl::VIDEO_SETTINGS::BRIGHTNESS; return true;
        case ZedMini::Setting::Contrast: *out = sl::VIDEO_SETTINGS::CONTRAST; return true;
        case ZedMini::Setting::Saturation: *out = sl::VIDEO_SETTINGS::SATURATION; return true;
        case ZedMini::Setting::Sharpness: *out = sl::VIDEO_SETTINGS::SHARPNESS; return true;
        case ZedMini::Setting::Gain: *out = sl::VIDEO_SETTINGS::GAIN; return true;
        case ZedMini::Setting::Exposure: *out = sl::VIDEO_SETTINGS::EXPOSURE; return true;
        case ZedMini::Setting::WhiteBalanceTemperature:
            *out = sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE;
            return true;
        case ZedMini::Setting::LedStatus: *out = sl::VIDEO_SETTINGS::LED_STATUS; return true;
    }
    return false;
}

}  // namespace

struct ZedMini::Impl {
    sl::Camera camera;
    sl::RuntimeParameters runtime;
    std::array<sl::Mat, 3> images;
    std::string error;
    unsigned int serial = 0;
    uint64_t timestamp_ns = 0;
    bool opened = false;
    bool has_frame = false;
};

ZedMini::ZedMini() : impl_(new Impl) {}

ZedMini::~ZedMini() {
    close();
    delete impl_;
}

bool ZedMini::open(unsigned int serial_number) {
    close();
    sl::InitParameters init;
    init.camera_resolution = sl::RESOLUTION::HD720;
    init.camera_fps = 60;
    init.depth_mode = sl::DEPTH_MODE::NONE;
    init.sdk_verbose = false;
    if (serial_number) init.input.setFromSerialNumber(serial_number);

    const auto status = impl_->camera.open(init);
    if (status != sl::ERROR_CODE::SUCCESS) {
        impl_->error = error_text(status);
        return false;
    }
    const auto info = impl_->camera.getCameraInformation();
    if (info.camera_model != sl::MODEL::ZED_M) {
        impl_->camera.close();
        impl_->error = "connected camera is not a ZED Mini";
        return false;
    }
    impl_->serial = info.serial_number;
    impl_->opened = true;
    impl_->error.clear();
    return true;
}

void ZedMini::close() {
    if (impl_->opened) impl_->camera.close();
    impl_->opened = false;
    impl_->has_frame = false;
    impl_->serial = 0;
    impl_->timestamp_ns = 0;
}

bool ZedMini::is_open() const { return impl_->opened; }

bool ZedMini::grab() {
    if (!impl_->opened) {
        impl_->error = "camera is not open";
        return false;
    }
    impl_->has_frame = false;
    const auto status = impl_->camera.grab(impl_->runtime);
    if (status != sl::ERROR_CODE::SUCCESS) {
        impl_->error = error_text(status);
        return false;
    }
    impl_->timestamp_ns = impl_->camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
    impl_->has_frame = true;
    impl_->error.clear();
    return true;
}

bool ZedMini::retrieve(View view, ImageView* out) {
    if (!out) {
        impl_->error = "output is null";
        return false;
    }
    *out = {};
    if (!impl_->has_frame) {
        impl_->error = "grab() must succeed before retrieve()";
        return false;
    }

    size_t index = 0;
    sl::VIEW zed_view = sl::VIEW::LEFT;
    switch (view) {
        case View::Left: break;
        case View::Right: index = 1; zed_view = sl::VIEW::RIGHT; break;
        case View::SideBySide: index = 2; zed_view = sl::VIEW::SIDE_BY_SIDE; break;
    }

    auto& image = impl_->images[index];
    const auto status = impl_->camera.retrieveImage(image, zed_view, sl::MEM::CPU);
    if (status != sl::ERROR_CODE::SUCCESS) {
        impl_->error = error_text(status);
        return false;
    }
    out->data = reinterpret_cast<const uint8_t*>(image.getPtr<sl::uchar1>(sl::MEM::CPU));
    out->stride_bytes = image.getStepBytes(sl::MEM::CPU);
    out->size_bytes = out->stride_bytes * image.getHeight();
    out->width = static_cast<int>(image.getWidth());
    out->height = static_cast<int>(image.getHeight());
    out->channels = static_cast<int>(image.getChannels());
    out->timestamp_ns = impl_->timestamp_ns;
    impl_->error.clear();
    return true;
}

bool ZedMini::set_setting(Setting setting, int value) {
    if (!impl_->opened) {
        impl_->error = "camera is not open";
        return false;
    }
    sl::VIDEO_SETTINGS zed_setting;
    if (!to_zed_setting(setting, &zed_setting)) {
        impl_->error = "unknown camera setting";
        return false;
    }
    const auto status = impl_->camera.setCameraSettings(zed_setting, value);
    if (status != sl::ERROR_CODE::SUCCESS) {
        impl_->error = error_text(status);
        return false;
    }
    impl_->error.clear();
    return true;
}

bool ZedMini::get_setting(Setting setting, int* value) {
    if (!value) {
        impl_->error = "output is null";
        return false;
    }
    if (!impl_->opened) {
        impl_->error = "camera is not open";
        return false;
    }
    sl::VIDEO_SETTINGS zed_setting;
    if (!to_zed_setting(setting, &zed_setting)) {
        impl_->error = "unknown camera setting";
        return false;
    }
    const auto status = impl_->camera.getCameraSettings(zed_setting, *value);
    if (status != sl::ERROR_CODE::SUCCESS) {
        impl_->error = error_text(status);
        return false;
    }
    impl_->error.clear();
    return true;
}

unsigned int ZedMini::serial_number() const { return impl_->serial; }

const std::string& ZedMini::last_error() const { return impl_->error; }

}  // namespace f32c::camera

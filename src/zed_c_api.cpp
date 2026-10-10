#include "f32c/camera_c_api.h"

#include "f32c/camera/zed_mini.hpp"

struct f32c_zed_camera {
    f32c::camera::ZedMini camera;
};

namespace {

bool to_view(unsigned view, f32c::camera::ZedMini::View* out) {
    if (!out) return false;
    switch (view) {
        case F32C_ZED_VIEW_LEFT: *out = f32c::camera::ZedMini::View::Left; return true;
        case F32C_ZED_VIEW_RIGHT: *out = f32c::camera::ZedMini::View::Right; return true;
        case F32C_ZED_VIEW_SIDE_BY_SIDE: *out = f32c::camera::ZedMini::View::SideBySide; return true;
    }
    return false;
}

bool to_setting(unsigned setting, f32c::camera::ZedMini::Setting* out) {
    if (!out) return false;
    if (setting > F32C_ZED_SETTING_LED_STATUS) return false;
    *out = static_cast<f32c::camera::ZedMini::Setting>(setting);
    return true;
}

}  // namespace

f32c_zed_camera* f32c_zed_create(void) { return new f32c_zed_camera; }

void f32c_zed_destroy(f32c_zed_camera* camera) { delete camera; }

int f32c_zed_open(f32c_zed_camera* camera, unsigned serial_number) {
    return camera && camera->camera.open(serial_number) ? 1 : 0;
}

void f32c_zed_close(f32c_zed_camera* camera) {
    if (camera) camera->camera.close();
}

int f32c_zed_is_open(const f32c_zed_camera* camera) {
    return camera && camera->camera.is_open() ? 1 : 0;
}

int f32c_zed_grab(f32c_zed_camera* camera) {
    return camera && camera->camera.grab() ? 1 : 0;
}

int f32c_zed_retrieve(f32c_zed_camera* camera, unsigned view, f32c_zed_image_view* out) {
    if (!camera || !out) return 0;
    f32c::camera::ZedMini::View zed_view;
    if (!to_view(view, &zed_view)) return 0;
    f32c::camera::ZedMini::ImageView image;
    if (!camera->camera.retrieve(zed_view, &image)) return 0;
    out->data = image.data;
    out->size_bytes = image.size_bytes;
    out->stride_bytes = image.stride_bytes;
    out->width = image.width;
    out->height = image.height;
    out->channels = image.channels;
    out->timestamp_ns = image.timestamp_ns;
    return 1;
}

int f32c_zed_set_setting(f32c_zed_camera* camera, unsigned setting, int value) {
    if (!camera) return 0;
    f32c::camera::ZedMini::Setting zed_setting;
    return to_setting(setting, &zed_setting) && camera->camera.set_setting(zed_setting, value) ? 1 : 0;
}

int f32c_zed_get_setting(f32c_zed_camera* camera, unsigned setting, int* value) {
    if (!camera) return 0;
    f32c::camera::ZedMini::Setting zed_setting;
    return to_setting(setting, &zed_setting) && camera->camera.get_setting(zed_setting, value) ? 1 : 0;
}

unsigned f32c_zed_serial_number(const f32c_zed_camera* camera) {
    return camera ? camera->camera.serial_number() : 0;
}

const char* f32c_zed_last_error(const f32c_zed_camera* camera) {
    static const char* null_error = "camera handle is null";
    return camera ? camera->camera.last_error().c_str() : null_error;
}

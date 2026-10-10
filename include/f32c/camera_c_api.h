#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct f32c_zed_camera f32c_zed_camera;

typedef struct {
    const uint8_t* data;
    size_t size_bytes;
    size_t stride_bytes;
    int width;
    int height;
    int channels;
    uint64_t timestamp_ns;
} f32c_zed_image_view;

enum {
    F32C_ZED_VIEW_LEFT = 0,
    F32C_ZED_VIEW_RIGHT = 1,
    F32C_ZED_VIEW_SIDE_BY_SIDE = 2,
};

enum {
    F32C_ZED_SETTING_BRIGHTNESS = 0,
    F32C_ZED_SETTING_CONTRAST = 1,
    F32C_ZED_SETTING_SATURATION = 2,
    F32C_ZED_SETTING_SHARPNESS = 3,
    F32C_ZED_SETTING_GAIN = 4,
    F32C_ZED_SETTING_EXPOSURE = 5,
    F32C_ZED_SETTING_WHITE_BALANCE_TEMPERATURE = 6,
    F32C_ZED_SETTING_LED_STATUS = 7,
};

f32c_zed_camera* f32c_zed_create(void);
void f32c_zed_destroy(f32c_zed_camera* camera);
int f32c_zed_open(f32c_zed_camera* camera, unsigned serial_number);
void f32c_zed_close(f32c_zed_camera* camera);
int f32c_zed_is_open(const f32c_zed_camera* camera);
int f32c_zed_grab(f32c_zed_camera* camera);
int f32c_zed_retrieve(f32c_zed_camera* camera, unsigned view, f32c_zed_image_view* out);
int f32c_zed_set_setting(f32c_zed_camera* camera, unsigned setting, int value);
int f32c_zed_get_setting(f32c_zed_camera* camera, unsigned setting, int* value);
unsigned f32c_zed_serial_number(const f32c_zed_camera* camera);
const char* f32c_zed_last_error(const f32c_zed_camera* camera);

#ifdef __cplusplus
}
#endif

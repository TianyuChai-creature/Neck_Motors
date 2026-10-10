#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct f32c_gimbal f32c_gimbal;

typedef struct {
    const char* device; /* NULL or empty: scan serial devices */
    int baud;
    unsigned yaw_id;
    unsigned pitch_id;
    int travel_rpm;
    unsigned accel;
    unsigned yaw_speed_kp;
    unsigned yaw_speed_ki;
    unsigned yaw_pos_kp;
    unsigned yaw_pos_ki;
    unsigned pitch_speed_kp;
    unsigned pitch_speed_ki;
    unsigned pitch_pos_kp;
    unsigned pitch_pos_ki;
} f32c_config;

void f32c_config_defaults(f32c_config* out);
f32c_gimbal* f32c_open_config(const f32c_config* config);
/* Existing shorthand; NULL device scans. Returns NULL on open/ping failure. */
f32c_gimbal* f32c_open(const char* device, unsigned yaw_id, unsigned pitch_id);
void f32c_close(f32c_gimbal* g);
/* For a failed open pass NULL; otherwise pass the live handle. */
const char* f32c_last_error(const f32c_gimbal* g);

int f32c_ping(f32c_gimbal* g);
int f32c_enable(f32c_gimbal* g);
int f32c_disable(f32c_gimbal* g);
/* axis 0: yaw, 1: pitch. Requires enabled motors; volatile gains only, each <= 256. */
int f32c_set_pi(f32c_gimbal* g, unsigned axis, unsigned speed_kp, unsigned speed_ki,
               unsigned pos_kp, unsigned pos_ki);
int f32c_set_deg(f32c_gimbal* g, double yaw_deg, double pitch_deg);
int f32c_set_yaw_deg(f32c_gimbal* g, double yaw_deg);
int f32c_set_pitch_deg(f32c_gimbal* g, double pitch_deg);
int f32c_read(f32c_gimbal* g, double* yaw_deg, double* pitch_deg);
/* Sets the current physical pose as zero and saves it to both motors. */
int f32c_set_zero(f32c_gimbal* g);
int f32c_start_log(f32c_gimbal* g, const char* path);
void f32c_stop_log(f32c_gimbal* g);

#ifdef __cplusplus
}
#endif

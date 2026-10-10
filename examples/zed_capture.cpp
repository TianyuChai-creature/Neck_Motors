#include "f32c/camera/zed_mini.hpp"

#include <cstdio>

int main() {
    using Camera = f32c::camera::ZedMini;
    Camera camera;
    if (!camera.open()) {
        std::fprintf(stderr, "open failed: %s\n", camera.last_error().c_str());
        return 1;
    }
    if (!camera.grab()) {
        std::fprintf(stderr, "grab failed: %s\n", camera.last_error().c_str());
        return 1;
    }

    const struct {
        const char* name;
        Camera::View view;
        int width;
    } views[] = {
        {"left", Camera::View::Left, 1280},
        {"right", Camera::View::Right, 1280},
        {"side_by_side", Camera::View::SideBySide, 2560},
    };

    for (const auto& expected : views) {
        Camera::ImageView image;
        if (!camera.retrieve(expected.view, &image)) {
            std::fprintf(stderr, "%s retrieve failed: %s\n", expected.name, camera.last_error().c_str());
            return 1;
        }
        std::printf("%s %dx%d BGRA8 timestamp=%llu bytes=%zu\n", expected.name,
                    image.width, image.height,
                    static_cast<unsigned long long>(image.timestamp_ns), image.size_bytes);
        if (!image.data || image.width != expected.width || image.height != 720 || image.channels != 4) {
            std::fprintf(stderr, "%s frame contract failed\n", expected.name);
            return 1;
        }
    }
    std::printf("zed mini online serial=%u\n", camera.serial_number());
    return 0;
}

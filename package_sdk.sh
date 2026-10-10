#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "$0")" && pwd)
python_bin=${F32C_PYTHON:-python3}
reuse_wheel=""
case "${1:-}" in
    "") ;;
    --reuse-native-wheel)
        if [ "$#" -ne 2 ]; then
            echo "usage: $0 [--reuse-native-wheel PATH]" >&2
            exit 1
        fi
        reuse_wheel=$2
        ;;
    --help)
        echo "usage: $0 [--reuse-native-wheel PATH]"
        echo "default: rebuild native SDK with CMake and ZED development dependencies"
        echo "reuse: verify and reuse the repository's 0.1.1 Linux x86_64 native SDK"
        exit 0
        ;;
    *)
        echo "usage: $0 [--reuse-native-wheel PATH]" >&2
        exit 1
        ;;
esac
if ! "$python_bin" -m pip --version >/dev/null 2>&1; then
    echo "set F32C_PYTHON to a Python with pip, setuptools, and wheel" >&2
    exit 1
fi

tmp_dir=$(mktemp -d /tmp/f32c-package-XXXXXX)
trap 'rm -rf "$tmp_dir"' EXIT
src_dir="$tmp_dir/python-src"
mkdir -p "$root_dir/wheel" "$src_dir/python/f32c" "$src_dir/docs" "$tmp_dir/out"

if [ -n "$reuse_wheel" ]; then
    "$python_bin" "$root_dir/tests/test_native_reuse.py"
    "$python_bin" "$root_dir/tools/stage_native_sdk.py" "$reuse_wheel" "$src_dir/python/f32c/native"
    native_lib_dir="$src_dir/python/f32c/native/lib"
    mkdir -p "$tmp_dir/tests"
    for test_name in protocol serial_log sdk_api; do
        "${CXX:-g++}" -std=c++17 -UNDEBUG -I"$root_dir/include" -I"$root_dir/src" \
            "$root_dir/tests/test_${test_name}.cpp" -L"$native_lib_dir" \
            -Wl,-rpath,"$native_lib_dir" -lf32c -lutil -o "$tmp_dir/tests/test_${test_name}"
        "$tmp_dir/tests/test_${test_name}"
    done
else
    cmake -S "$root_dir" -B "$tmp_dir/build" -DF32C_BUILD_ZED=ON \
        -DCMAKE_INSTALL_PREFIX="$src_dir/python/f32c/native" -DCMAKE_INSTALL_LIBDIR=lib -Wno-dev
    cmake --build "$tmp_dir/build" -j2
    ctest --test-dir "$tmp_dir/build" --output-on-failure
    cmake --install "$tmp_dir/build"
    native_lib_dir="$src_dir/python/f32c/native/lib"
fi
PYTHONPATH="$root_dir/python" F32C_LIBRARY_DIR="$native_lib_dir" "$python_bin" "$root_dir/tests/test_python_sdk.py"
F32C_LIBRARY_DIR="$native_lib_dir" "$python_bin" "$root_dir/tests/test_runtime_pi.py"
"$python_bin" "$root_dir/tests/test_tuning.py"
"$python_bin" "$root_dir/tests/test_tracking.py"

cp "$root_dir/pyproject.toml" "$root_dir/setup.py" "$src_dir/"
cp "$root_dir/python/f32c/"*.py "$src_dir/python/f32c/"
cp "$root_dir/docs/SDK使用说明.md" "$src_dir/docs/"
"$python_bin" -m pip wheel "$src_dir" --no-deps --no-build-isolation -w "$tmp_dir/out"

mv "$tmp_dir/out/"*.whl "$root_dir/wheel/"
cp "$root_dir/docs/SDK使用说明.md" "$root_dir/wheel/SDK使用说明.md"
echo "SDK wheel written to $root_dir/wheel"

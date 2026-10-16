#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "$0")" && pwd)
python_bin=${F32C_PYTHON:-python3}
if [ "$#" -ne 0 ]; then
    echo "usage: $0 (rebuilds the motor SDK with CMake)" >&2
    exit 1
fi
if ! "$python_bin" -m pip --version >/dev/null 2>&1; then
    echo "set F32C_PYTHON to a Python with pip, setuptools, and wheel" >&2
    exit 1
fi

tmp_dir=$(mktemp -d /tmp/f32c-package-XXXXXX)
trap 'rm -rf "$tmp_dir"' EXIT
src_dir="$tmp_dir/python-src"
mkdir -p "$root_dir/wheel" "$src_dir/python/f32c" "$src_dir/docs" "$tmp_dir/out"

cmake -S "$root_dir" -B "$tmp_dir/build" -DF32C_BUILD_ZED=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$src_dir/python/f32c/native" -DCMAKE_INSTALL_LIBDIR=lib -Wno-dev
cmake --build "$tmp_dir/build" -j2
ctest --test-dir "$tmp_dir/build" --output-on-failure
cmake --install "$tmp_dir/build"
native_lib_dir="$src_dir/python/f32c/native/lib"
PYTHONPATH="$root_dir/python" F32C_LIBRARY_DIR="$native_lib_dir" "$python_bin" "$root_dir/tests/test_python_sdk.py"
F32C_LIBRARY_DIR="$native_lib_dir" "$python_bin" "$root_dir/tests/test_runtime_pi.py"
"$python_bin" "$root_dir/tests/test_tuning.py"
"$python_bin" "$root_dir/tests/test_tracking.py"
PYTHONPATH="$root_dir/python" F32C_LIBRARY_DIR="$native_lib_dir" \
    "$python_bin" "$root_dir/tests/test_controller.py"
PYTHONPATH="$root_dir/python" F32C_LIBRARY_DIR="$native_lib_dir" \
    "$python_bin" "$root_dir/tests/test_receive.py"

cp "$root_dir/pyproject.toml" "$root_dir/setup.py" "$src_dir/"
cp "$root_dir/python/f32c/"*.py "$src_dir/python/f32c/"
cp "$root_dir/docs/SDK使用说明.md" "$src_dir/docs/"
"$python_bin" -m pip wheel "$src_dir" --no-deps --no-build-isolation -w "$tmp_dir/out"
"$python_bin" "$root_dir/tests/test_wheel.py" "$tmp_dir/out/"*.whl

mv "$tmp_dir/out/"*.whl "$root_dir/wheel/"
cp "$root_dir/docs/SDK使用说明.md" "$root_dir/wheel/SDK使用说明.md"
echo "SDK wheel written to $root_dir/wheel"

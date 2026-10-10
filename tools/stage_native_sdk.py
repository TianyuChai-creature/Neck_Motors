"""Stage the unchanged 0.1.1 native SDK for a Python-only patch release.

Refuse reuse if either the known release archive or any native source changed.
Only version metadata and the installed manual are refreshed; libraries, public
headers and CMake target exports keep their original bytes.
"""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import platform
import re
import zipfile


ROOT = Path(__file__).resolve().parents[1]
BASE_VERSION = "0.1.1"
BASE_WHEEL_SHA256 = "fd0431618f3e30b8e426a5b3622cb915b87c667e543189885170f51ea2c61701"
BASE_SOURCE_SHA256 = "0d9d8adc5fa7d1d46fea8923ae031e2c01bf33cd1eea41a5fed3b9dc552c5297"


def native_source_sha256(root: Path) -> str:
    files = [p for directory in ("src", "include", "cmake")
             for p in (root / directory).rglob("*") if p.is_file()]
    files.append(root / "CMakeLists.txt")
    digest = hashlib.sha256()
    for path in sorted(files, key=lambda p: p.relative_to(root).as_posix()):
        contents = path.read_bytes()
        if path.name == "CMakeLists.txt":
            contents = re.sub(rb"project\(f32c_sdk VERSION [0-9.]+ LANGUAGES CXX\)",
                              b"project(f32c_sdk VERSION @VERSION@ LANGUAGES CXX)", contents)
        digest.update(path.relative_to(root).as_posix().encode() + b"\0" + contents + b"\0")
    return digest.hexdigest()


def stage(wheel_path: Path, destination: Path) -> None:
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        raise ValueError("native reuse is supported only on Linux x86_64")
    if hashlib.sha256(wheel_path.read_bytes()).hexdigest() != BASE_WHEEL_SHA256:
        raise ValueError("native reuse requires the unmodified repository 0.1.1 wheel")
    if native_source_sha256(ROOT) != BASE_SOURCE_SHA256:
        raise ValueError("native sources changed; rebuild with CMake and ZED instead of reusing libraries")
    if destination.exists():
        raise ValueError("native staging destination must not already exist")
    match = re.search(r'^version = "([0-9]+\.[0-9]+\.[0-9]+)"$',
                      (ROOT / "pyproject.toml").read_text(), re.MULTILINE)
    if match is None:
        raise ValueError("expected a three-part release version in pyproject.toml")
    version = match[1]
    if f"project(f32c_sdk VERSION {version} LANGUAGES CXX)" not in (ROOT / "CMakeLists.txt").read_text():
        raise ValueError("Python and CMake SDK versions must match")
    prefix = f"f32c_neck_motors-{BASE_VERSION}.data/purelib/f32c/native/"
    payload = {}
    with zipfile.ZipFile(wheel_path) as archive:
        for name in archive.namelist():
            if name.startswith(prefix) and not name.endswith("/"):
                relative = PurePosixPath(name[len(prefix):])
                if relative.is_absolute() or ".." in relative.parts:
                    raise ValueError("invalid native archive path")
                payload[relative.as_posix()] = archive.read(name)
    required = {"lib/libf32c.so", "lib/libf32c_zed.so", "include/f32c/c_api.h",
                "include/f32c/gimbal.hpp", "lib/cmake/f32c/f32cConfigVersion.cmake"}
    if not required.issubset(payload):
        raise ValueError("release wheel has an incomplete native SDK")
    version_path = "lib/cmake/f32c/f32cConfigVersion.cmake"
    payload[version_path] = payload[version_path].replace(BASE_VERSION.encode(), version.encode())
    payload["share/doc/f32c/SDK使用说明.md"] = (ROOT / "docs/SDK使用说明.md").read_bytes()
    build_info = {
        "sdk_version": version,
        "native_rebuilt": False,
        "native_source_version": BASE_VERSION,
        "native_source_sha256": BASE_SOURCE_SHA256,
        "source_wheel": f"f32c_neck_motors-{BASE_VERSION}-py3-none-linux_x86_64.whl",
        "source_wheel_sha256": BASE_WHEEL_SHA256,
        "native_files_sha256": {name: hashlib.sha256(data).hexdigest()
                                for name, data in sorted(payload.items())},
    }
    payload["share/doc/f32c/build-info.json"] = (
        json.dumps(build_info, indent=2, ensure_ascii=False) + "\n").encode()
    for name, contents in payload.items():
        output = destination / name
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(contents)
    print(f"Verified unchanged {BASE_VERSION} native SDK; staged for Python release {version}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    try:
        stage(args.wheel, args.destination)
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        parser.exit(1, f"{exc}\n")

"""Validate the released wheel, including the motor-only SDK boundary."""

import base64
import csv
from email import message_from_string
import hashlib
import io
from pathlib import Path
import re
import sys
import zipfile


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(path: Path) -> None:
    version = re.search(r'^version = "([^"]+)"$',
                        (ROOT / "pyproject.toml").read_text(), re.MULTILINE)[1]
    package = f"f32c_neck_motors-{version}"
    prefix = f"{package}.data/purelib/f32c/"
    manual = (ROOT / "docs/SDK使用说明.md").read_bytes()
    with zipfile.ZipFile(path) as archive:
        require(archive.testzip() is None, "wheel has a corrupt ZIP entry")
        names = set(archive.namelist())
        require(not any("zed" in n.lower() or "camera" in n.lower() for n in names),
                "camera component included in the motor wheel")
        libraries = {n for n in names if n.endswith(".so")}
        require(libraries == {prefix + "native/lib/libf32c.so"}, "unexpected native libraries")
        for source in (ROOT / "python/f32c").glob("*.py"):
            require(archive.read(prefix + source.name) == source.read_bytes(),
                    f"Python module missing or out of date: {source.name}")
        require(archive.read(prefix + "native/share/doc/f32c/SDK使用说明.md") == manual,
                "installed manual differs from the source manual")
        metadata = message_from_string(archive.read(f"{package}.dist-info/METADATA").decode())
        require(metadata["Version"] == version, "wheel version differs from source version")
        require(metadata.get_payload().rstrip() == manual.decode().rstrip(),
                "wheel metadata description differs from the manual")
        records = csv.reader(io.StringIO(archive.read(f"{package}.dist-info/RECORD").decode()))
        recorded = set()
        for name, encoded, size in records:
            recorded.add(name)
            if encoded:
                data = archive.read(name)
                digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).decode().rstrip("=")
                require(encoded == f"sha256={digest}" and len(data) == int(size),
                        f"invalid wheel RECORD entry: {name}")
        require(recorded == names, "wheel RECORD does not cover all archive files")
        for name in names:
            if name.endswith(".cmake"):
                contents = archive.read(name).decode().lower()
                require("zed" not in contents and "camera" not in contents,
                        f"camera target remains in CMake metadata: {name}")
        for header in ("gimbal.hpp", "c_api.h"):
            require(archive.read(prefix + "native/include/f32c/" + header) ==
                    (ROOT / "include/f32c" / header).read_bytes(),
                    f"installed public header differs from source: {header}")
    print(f"Motor-only wheel contents, source modules, manual and RECORD verified: {path.name}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_wheel.py PATH.whl")
    validate(Path(sys.argv[1]))

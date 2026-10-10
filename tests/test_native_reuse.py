"""Check archive fidelity and refuse incompatible native payload reuse."""

import json
from pathlib import Path
import platform
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import stage_native_sdk


@unittest.skipUnless(platform.system() == "Linux" and platform.machine() == "x86_64",
                     "baseline archive is Linux x86_64")
class NativeReuseTests(unittest.TestCase):
    wheel = ROOT / "wheel/f32c_neck_motors-0.1.1-py3-none-linux_x86_64.whl"

    def test_preserves_every_native_payload_except_version_and_manual(self):
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary) / "native"
            stage_native_sdk.stage(self.wheel, destination)
            prefix = "f32c_neck_motors-0.1.1.data/purelib/f32c/native/"
            changed = {"share/doc/f32c/SDK使用说明.md", "lib/cmake/f32c/f32cConfigVersion.cmake"}
            with zipfile.ZipFile(self.wheel) as archive:
                for name in archive.namelist():
                    if name.startswith(prefix) and not name.endswith("/"):
                        relative = name[len(prefix):]
                        if relative not in changed:
                            self.assertEqual((destination / relative).read_bytes(), archive.read(name))
            self.assertEqual((destination / "share/doc/f32c/SDK使用说明.md").read_bytes(),
                             (ROOT / "docs/SDK使用说明.md").read_bytes())
            info = json.loads((destination / "share/doc/f32c/build-info.json").read_text())
            self.assertFalse(info["native_rebuilt"])
            self.assertEqual(info["native_source_version"], "0.1.1")
            version_file = (destination / "lib/cmake/f32c/f32cConfigVersion.cmake").read_text()
            self.assertIn(f'set(PACKAGE_VERSION "{info["sdk_version"]}")', version_file)

    def test_tampered_archive_is_rejected_before_writing(self):
        with tempfile.TemporaryDirectory() as temporary:
            wheel = Path(temporary) / "tampered.whl"
            wheel.write_bytes(self.wheel.read_bytes() + b"tampered")
            destination = Path(temporary) / "native"
            with self.assertRaisesRegex(ValueError, "unmodified"):
                stage_native_sdk.stage(wheel, destination)
            self.assertFalse(destination.exists())

    def test_modified_native_source_is_rejected_before_writing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for directory in ("src", "include", "cmake"):
                shutil.copytree(ROOT / directory, root / directory)
            shutil.copyfile(ROOT / "CMakeLists.txt", root / "CMakeLists.txt")
            with (root / "src/gimbal.cpp").open("a") as output:
                output.write("\n// Native implementation changed.\n")
            destination = root / "native"
            with patch.object(stage_native_sdk, "ROOT", root):
                with self.assertRaisesRegex(ValueError, "sources changed"):
                    stage_native_sdk.stage(self.wheel, destination)
            self.assertFalse(destination.exists())


if __name__ == "__main__":
    unittest.main()

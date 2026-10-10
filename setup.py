from pathlib import Path

from setuptools import setup
from wheel.bdist_wheel import bdist_wheel


class PlatformWheel(bdist_wheel):
    def finalize_options(self):
        super().finalize_options()
        self.root_is_pure = False

    def get_tag(self):
        _, _, platform = super().get_tag()
        return "py3", "none", platform


native = Path("python/f32c/native")
if not (native / "lib/libf32c.so").is_file():
    raise RuntimeError("native SDK is missing; build the wheel with package_sdk.sh")

setup(
    cmdclass={"bdist_wheel": PlatformWheel},
    package_data={"f32c": [str(path.relative_to("python/f32c"))
                           for path in native.rglob("*") if path.is_file()]},
)

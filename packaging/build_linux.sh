#!/bin/sh
# Build the Linux single-file executable and the AppImage from tools/ui.py.
# Needs: pip install pyinstaller PySide6; appimagetool in PATH (or APPIMAGETOOL=/path/to/it).
set -e
cd "$(dirname "$0")/.."
OUT=${OUT:-build}
TOOL=${APPIMAGETOOL:-appimagetool}
EXCLUDES="--exclude-module tkinter --exclude-module PySide6.QtNetwork --exclude-module PySide6.QtQml
 --exclude-module PySide6.QtQuick --exclude-module PySide6.QtPdf --exclude-module PySide6.QtSvg
 --exclude-module PySide6.QtOpenGL"
rm -rf "$OUT" && mkdir -p "$OUT"

# one folder (feeds the AppImage) and one file
pyinstaller --noconfirm --windowed --name f32c-neck-ui --paths . $EXCLUDES \
  --distpath "$OUT/dist" --workpath "$OUT/work" --specpath "$OUT/spec" tools/ui.py
pyinstaller --noconfirm --onefile --windowed --name f32c-neck-ui-linux-x86_64 --paths . $EXCLUDES \
  --distpath "$OUT/dist1" --workpath "$OUT/work1" --specpath "$OUT/spec1" tools/ui.py

# AppDir -> AppImage (--appimage-extract-and-run: works without FUSE)
mkdir -p "$OUT/AppDir/usr/bin"
cp -r "$OUT/dist/f32c-neck-ui" "$OUT/AppDir/usr/bin/"
cp packaging/AppRun packaging/f32c-neck-ui.desktop packaging/f32c-neck-ui.png "$OUT/AppDir/"
ARCH=x86_64 "$TOOL" --appimage-extract-and-run --no-appstream "$OUT/AppDir" "$OUT/F32C-Neck-UI-x86_64.AppImage"
cp "$OUT/dist1/f32c-neck-ui-linux-x86_64" "$OUT/"
echo "built in $OUT/"

#!/usr/bin/env bash
# Build Rockbox for the iPod classic (6th gen) with the car gauges patches.
# Needs Rockbox's ARM toolchain (rockbox/tools/rockboxdev.sh --target=a) on PATH.
# Output in out/: rockbox.zip, bootloader-ipod6g.ipod and the mks5lboot installer.
set -euo pipefail
cd "$(dirname "$0")"
BASE=7d090f24b990ef54e3f9dc7eb2edba4865605471   # upstream commit the patches apply to
ROOT=$PWD

if [ ! -d rockbox ]; then
    git clone https://github.com/Rockbox/rockbox.git rockbox
fi
cd rockbox
if ! git rev-parse -q --verify car-gauges >/dev/null; then
    git checkout -q -b car-gauges "$BASE"
    git -c user.name=build -c user.email=build@localhost am -q "$ROOT"/patches/*.patch
fi
git checkout -q car-gauges

mkdir -p build-6g build-boot "$ROOT/out"
(cd build-6g && ../tools/configure --target=ipod6g --type=n </dev/null >/dev/null &&
    make -j"$(nproc)" >/dev/null && make zip >/dev/null && cp rockbox.zip "$ROOT/out/")
(cd build-boot && ../tools/configure --target=ipod6g --type=b </dev/null >/dev/null &&
    make -j"$(nproc)" >/dev/null && cp bootloader-ipod6g.ipod "$ROOT/out/")
(cd utils/mks5lboot && make >/dev/null && cp mks5lboot "$ROOT/out/")
ls -l "$ROOT/out"

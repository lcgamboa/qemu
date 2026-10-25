#!/usr/bin/env bash

set -euo pipefail

TARGET=${TARGET:-xtensa-softmmu}
VERSION=${VERSION:-dev}

echo DBG
./configure --help

# No -Werror/werror here: MSYS2 ships a much newer GCC (16.x) than this QEMU
# branch, and QEMU enables werror by default for git builds. New compiler
# warnings (e.g. -Wunused-but-set-variable, -Wdiscarded-qualifiers) would then
# break the build for reasons unrelated to the code being built.
# Same rationale as macOS.
./configure \
    --bindir=bin \
    --datadir=share/qemu \
    --enable-gcrypt \
    --enable-sdl \
    --enable-pixman \
    --enable-slirp \
    --enable-stack-protector \
    --prefix=${PWD}/install/qemu \
    --static \
    --target-list=${TARGET} \
    --with-pkgversion="${VERSION}" \
    --with-suffix="" \
    --without-default-features \
    -Dwerror=false \
|| { cat meson-logs/meson-log.txt && false; }


# Fix: pkg-config for libgcrypt outputs incorrect paths for libiconv and libintl:
# - Unix-style paths (/ucrt64/lib/...) instead of Windows paths (D:/a/_temp/msys64/ucrt64/lib/...)
# - Dynamic import libraries (.dll.a) instead of static libraries (.a)
# We need to fix both issues in build.ninja for the static build to work correctly.
MSYS_BASE=$(cygpath -w / | sed 's/\\/\//g')
MINGW_PREFIX=${MINGW_PREFIX:-/ucrt64}
sed -i "s|${MINGW_PREFIX}/lib/libintl.dll.a|${MSYS_BASE}${MINGW_PREFIX}/lib/libintl.a|g; s|${MINGW_PREFIX}/lib/libiconv.dll.a|${MSYS_BASE}${MINGW_PREFIX}/lib/libiconv.a|g" build/build.ninja

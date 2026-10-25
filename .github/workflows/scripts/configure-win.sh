#!/usr/bin/env bash

set -euo pipefail

TARGET=${TARGET:-xtensa-softmmu}
VERSION=${VERSION:-dev}

echo DBG
./configure --help

./configure \
    --bindir=bin \
    --datadir=share/qemu \
    --enable-gcrypt \
    --enable-sdl \
    --enable-pixman \
    --enable-slirp \
    --enable-stack-protector \
    --extra-cflags=-Werror \
    --prefix=${PWD}/install/qemu \
    --static \
    --target-list=${TARGET} \
    --with-pkgversion="${VERSION}" \
    --with-suffix="" \
    --without-default-features \
|| { cat meson-logs/meson-log.txt && false; }


# Fix: pkg-config for libgcrypt outputs incorrect paths for libiconv and libintl:
# - Unix-style paths (/ucrt64/lib/...) instead of Windows paths (D:/a/_temp/msys64/ucrt64/lib/...)
# - Dynamic import libraries (.dll.a) instead of static libraries (.a)
# We need to fix both issues in build.ninja for the static build to work correctly.
MSYS_BASE=$(cygpath -w / | sed 's/\\/\//g')
MINGW_PREFIX=${MINGW_PREFIX:-/ucrt64}
sed -i "s|${MINGW_PREFIX}/lib/libintl.dll.a|${MSYS_BASE}${MINGW_PREFIX}/lib/libintl.a|g; s|${MINGW_PREFIX}/lib/libiconv.dll.a|${MSYS_BASE}${MINGW_PREFIX}/lib/libiconv.a|g" build/build.ninja

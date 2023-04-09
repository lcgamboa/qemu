#!/bin/sh

set -e

target="arm-softmmu"

case "$(uname)" in
  Linux)
    ncpu="$(nproc)"
    ;;
  Darwin)
    ncpu="$(sysctl -n hw.physicalcpu)"
    ;;
esac

flags="$(./configure --help | perl -ne 'print if s/^  ([a-z][\w-]*) .*/\1/' | tail -n +2 | awk '{print "--disable-"$1}' ORS=' ')"

${2:-.}/configure --target-list=$target --extra-cflags=-fPIC --disable-slirp $flags --enable-tcg \
	--enable-system --disable-werror --disable-alsa  \
        --enable-debug --enable-debug-info 

make clean >/dev/null

# Build everything as usual
make "-j$ncpu" 

# Build a shared library, without softmmu/main.o and otherwise *exactly* the same
# flags.
cd build
rm -f qemu-system-arm
ninja -d keeprsp
#dynamic
sed -i 's/qemu-system-arm.p\/softmmu_main.c.o//g' qemu-system-arm.rsp
sed -i 's/-o\ qemu-system-arm/-shared\ -o\ libqemu-stm32.so/g' qemu-system-arm.rsp
c++ -m64 -mcx16 -ggdb @qemu-system-arm.rsp


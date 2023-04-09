#!/bin/sh

set -e

target="xtensa-softmmu"

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
        --enable-debug --enable-debug-info \
        --enable-gcrypt --enable-slirp 
	#--enable-gtk

make clean >/dev/null

# Build everything as usual
make "-j$ncpu" 

# Build a shared library, without softmmu/main.o and otherwise *exactly* the same
# flags.
cd build
rm -f qemu-system-xtensa
ninja -v > qemu-system-xtensa.rsp
sed -i '1d' qemu-system-xtensa.rsp
sed -i 's/\[2\/2\] c++ -m64 -mcx16//g' qemu-system-xtensa.rsp

#dynamic
sed -i 's/qemu-system-xtensa.p\/softmmu_main.c.o//g' qemu-system-xtensa.rsp
sed -i 's/-o\ qemu-system-xtensa/-shared\ -o\ libqemu-xtensa.so/g' qemu-system-xtensa.rsp
c++ -m64 -mcx16 -ggdb @qemu-system-xtensa.rsp





#!/bin/sh

set -e

target="xtensa-softmmu,riscv32-softmmu"

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
sed -i -n '$p' qemu-system-xtensa.rsp
CMD=$(sed  's/-o .*//' qemu-system-xtensa.rsp | sed 's/\[.\/.\] //g')
sed -i 's/.*-o /-o /' qemu-system-xtensa.rsp

#dynamic
sed -i 's/qemu-system-xtensa.p\/softmmu_main.c.o//g' qemu-system-xtensa.rsp
sed -i 's/-o\ qemu-system-xtensa/-shared\ -o\ libqemu-xtensa.so/g' qemu-system-xtensa.rsp
eval "$CMD -ggdb @qemu-system-xtensa.rsp"


rm -f qemu-system-riscv32
ninja -v > qemu-system-riscv32.rsp
sed -i -n '$p' qemu-system-riscv32.rsp
CMD=$(sed  's/-o .*//' qemu-system-riscv32.rsp | sed 's/\[.\/.\] //g')
sed -i 's/.*-o /-o /' qemu-system-riscv32.rsp


#dynamic
sed -i 's/qemu-system-riscv32.p\/softmmu_main.c.o//g' qemu-system-riscv32.rsp
sed -i 's/-o\ qemu-system-riscv32/-shared\ -o\ libqemu-riscv32.so/g' qemu-system-riscv32.rsp
eval "$CMD -ggdb @qemu-system-riscv32.rsp"


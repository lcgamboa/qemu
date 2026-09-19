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

if [ "$(uname)" = "Darwin" ]; then
  rm -rf .venv
  uv venv .venv
  source .venv/bin/activate
  uv pip install distlib
fi


flags="$(./configure --help | perl -ne 'print if s/^  ([a-z][\w-]*) .*/\1/' | tail -n +2 | awk '{print "--disable-"$1}' ORS=' ')"

${2:-.}/configure --target-list=$target --extra-cflags=-fPIC --disable-slirp $flags --enable-tcg \
	--enable-system --disable-werror --disable-alsa  \
        --enable-debug --enable-debug-info 

make clean >/dev/null

# Build everything as usual
make "-j$ncpu" 

# Build a shared library, without softmmu/main.o on Linux and qemu-system-arm-unsigned.p/system_main.c.o on macOS
# and otherwise *exactly* the same flags
cd build
rm -f qemu-system-arm qemu-system-arm-unsigned
ninja -v -d keeprsp > qemu-system-arm_cmd.rsp
if [ "$(uname)" = "Darwin" ]; then
  # For macOS we extract not the last line, but the second line from the end
  # because the last line calls the script which signs the binary
  CMD=$(tail -n 2 qemu-system-arm_cmd.rsp | head -n 1)

  # Strip the ninja progress prefix
  CMD="${CMD#*] }"

  # Remove the qemu-system-arm-unsigned.p/system_main.c.o
  CMD="${CMD//qemu-system-arm-unsigned.p\/system_main.c.o/}"

  # Build a dynamic lib instead of an executable
  CMD="${CMD/-o qemu-system-arm-unsigned/-dynamiclib -o libqemu-stm32.dylib}"

  eval "$CMD"
else
  sed -i -n '$p' qemu-system-arm_cmd.rsp
  CMD=$(sed  's/\@.*//' qemu-system-arm_cmd.rsp | sed 's/\[.\/.\] //g')

  #dynamic
  sed -i 's/qemu-system-arm.p\/softmmu_main.c.o//g' qemu-system-arm.rsp
  sed -i 's/-o\ qemu-system-arm/-shared\ -o\ libqemu-stm32.so/g' qemu-system-arm.rsp
  eval "$CMD  -ggdb @qemu-system-arm.rsp"
fi



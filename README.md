# QEMU for use with PICSimLab

This fork contains a modified version of the [Espressif QEMU](https://github.com/espressif/qemu) used by the [PICSimLab](https://github.com/lcgamboa/picsimlab) simulator. This version has been altered to compile QEMU as a dynamic library using the [build_libqemu-esp32.sh](https://github.com/lcgamboa/qemu/blob/picsimlab-esp32/build_libqemu-esp32.sh) script.

Additionally, this fork adds support for WIFI from the [a159x36 QEMU fork](https://github.com/a159x36/qemu) to the [Espressif QEMU](https://github.com/espressif/qemu).

Official [QEMU README](https://github.com/lcgamboa/qemu/blob/picsimlab-esp32/README.rst)

# Differences from the Espressif QEMU fork

* Support for compiling as a dynamic library
* WIFI support (Station and SoftAP modes) for ESP32 and ESP32C3
* ESP-NOW protocol support for ESP32 and ESP32C3
* ESP32 an ESP32C3 IOMUX

# Using WIFI support

To enable WIFI support, pass the following option in the command line:

For ESP32: "-nic user,model=esp32_wifi":

```
qemu-system-xtensa -M esp32-picsimlab -drive file=flash_file.bin,if=mtd,format=raw \
  -drive file=esp32_file.efuse,if=none,format=raw,id=efuse \
  -global driver=nvram.esp32.efuse,property=drive,value=efuse \
  -serial stdio -gdb tcp::1234 -global driver=timer.esp32.timg,property=wdt_disable,value=true \
  -nic user,model=esp32_wifi,net=192.168.4.0/24,hostfwd=tcp::16555-192.168.4.15:80 
```

For ESP32C3 -nic user,model=esp32c3_wifi:
```
./qemu-system-riscv32 -M esp32c3-picsimlab -drive file=flash_file.bin,if=mtd,format=raw \
  -drive file=esp32c3.efuse,if=none,format=raw,id=efuse \
  -global driver=nvram.esp32c3.efuse,property=drive,value=efuse \
  -serial stdio -gdb tcp::1234 -icount shift=3,align=off,sleep=on \
  -global driver=timer.esp32c3.timg,property=wdt_disable,value=true \
  -nic user,model=esp32c3_wifi,net=192.168.4.0/24,hostfwd=tcp::16555-192.168.4.15:80
```

For use with ESP-NOW, configure the WIFI interface as socket mcast and ensure each device has a different MAC address in the .efuse file.
```
-nic socket,model=esp32_wifi,id=u1,mcast=230.0.0.1:1234
```

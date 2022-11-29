#!/bin/sh

while getopts r OPT
do
    case "${OPT}" in
        r) release_mode="true";;
    esac
done

if [ -n "$release_mode" ]
then
    echo "Building in release mode"
    cargo build --release --target=thumbv7em-none-eabihf
    arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/otomo-rs otomo-release.bin
    OTOMO_BIN="otomo-release.bin"
else
    echo "Building in debug mode"
    cargo build --target=thumbv7em-none-eabihf
    arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/debug/otomo-rs otomo-debug.bin
    OTOMO_BIN="otomo-debug.bin"
fi

openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg -c "program $OTOMO_BIN 0x8000000 reset exit"

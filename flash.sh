#!/bin/sh

cargo build --release --target=thumbv7em-none-eabihf

arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/otomo-rs otomo-rs.bin

st-flash write otomo-rs.bin 0x8000000

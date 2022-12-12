# otomo-rs
This started as a Rust port of some robot firmware that I was writing, but ended up being better than the original firmware.  Neat.  The concept is to receive instructions from a serial link, drive the robot, and eventually include closed-loop control to maintain heading and speed across four bad motors and even worse encoders.  Then I'll start working on some object detection.

## communication protocol
Comms are serialized in protobuf, (otomo-proto), using the prost crate.  I don't like prost.  It's forcing me to use an allocator which can impact performance.  The reason I'm using protobuf and not something like postcard is due to the serial link being connected to an HC05 bluetooth module, which _then_ communicates with an Android app.  Postcard isn't supported in Android (it's rust-native) so yeah, proto.

## internal architecture
This firmware uses rtic to handle concurrency across interrupts.  The serial isr will ingest the messages, then send them to the idle thread where the motor driving will be done.  Eventually I'll add an isr for a timer to query an IMU (and an isr for whatever that uses) as well as some handlers for the encoders.

The idea is to have as much of the processing logic outside of the main crate, but that'll come with size I think.

# TODO

- [X] get HC05 up and running
- [X] get communication running between phone and device
- [X] get protobuf messaging working
- [ ] clean up init stuff <- lol that'll probably never gonna happen
- [X] joystick to tank conversion
- [ ] read crappy encoders
- [ ] add PID library for speed control
- [ ] read from IMU to get heading
- [ ] heading-based input to control loop
- [X] read from laser/ultrasonic/IR distance sensor
- [ ] obstacle avoidance: don't drive in direction of obstacle

# Optional TODO
- [ ] log crate instead of defmt (configure through features)
- [ ] populate constants/thresholds at build time through config file
- [ ] messaging that doesn't obliterate heap usage and flash space
- [ ] better separation of hardware control and logic
- [ ] LCD screen
- [ ] SD card logs
- [ ] just...better robot chassis hardware.

# Notes on debugging
## Getting a defmt session running
```console
cargo run -- --connect-under-reset
```

## Getting GDB session running
in one shell:
```console 
$ openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg
```
And in another shell: 
```console 
$ gdb-multiarch target/thumbv7em-none-eabihf/debug/otomo-rs -ex "target extended-remote localhost:3333"
```

## Just flash it
```console
$ ./flash.sh # -r for release build
```

## Strange interactions
For whatever reason the Saleae logic analyzers cause the stlink to be disconnected when plugged in.  I have *no* idea why this is.

# Bibliography
https://howtomechatronics.com/tutorials/arduino/arduino-and-hc-05-bluetooth-module-tutorial/
https://robotics.stackexchange.com/questions/2011/how-to-calculate-the-right-and-left-speed-for-a-tank-like-rover

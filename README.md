# otomo-rs
This started as a Rust port of some robot firmware that I was writing, but ended up being better than the original firmware.  Neat.  The concept is to receive instructions from a serial link, drive the robot, and eventually include closed-loop control to maintain heading and speed across four bad motors and even worse encoders.  Then I'll start working on some object detection.

### hardware update 1
After using the "four bad motors and even worse encoders" for too long, I completely rebuilt the chassis to not be a literal toy.  It still looks like junk, just, more professional junk.  The base unit is now a single sheet of plywood, instead of the layered acryllic sheets from before (pictures soon!).  The reasoning for this was to make wire routing simpler, since I can now put all components on one flat plane instead of carving channels into layers of acrylic.  The original chassis was based on a toy (The DFRobot Baron) which could realistically only mount the original garbage motors.  The new chassis uses "shopping cart dynamics" where it has two powered wheels in the rear, and two casters in the front.  It is still a diff drive robot.  The new chassis was also intended to be extremely cheap, and constructable out of parts that could be found at any hardware store, with the exception of the motors.  I will inevitably want to add more/better hardware, so cheap and cheerful is the way to go.

## communication protocol
Comms are serialized in protobuf, (otomo-proto), using the prost crate.  I don't like prost.  It's forcing me to use an allocator which can impact performance.  The reason I'm using protobuf and not something like postcard is due to the serial link being connected to an HC05 bluetooth module, which _then_ communicates with an Android app.  Postcard isn't supported in Android (it's rust-native) so yeah, proto.

Message delimiting is accomplished using KISS-TNC.  It is a simple protocol used to denote the start and end of messages, and uses a simple frame-escape method to handle when a value in the message is the same as the frame delimiter.  See the kiss_encoding crate for details.  The crate in this workspace is an ultra-simplified version of the KISS protocol to work for this project on an embedded system.

## internal architecture

### scheduling 
This firmware uses rtic to handle concurrency across interrupts.  The serial isr will ingest the messages, then send them to the idle thread where the motor driving will be done.  Eventually I'll add an isr for a timer to query an IMU (and an isr for whatever that uses) as well as some handlers for the encoders.

### hardware abstraction
This firmware uses the otomo_hardware create (workspace) to move the hardware initialization outside of the init method in rtic.  This was done to keep file size down, but eventually it should turn into a semi-HAL.  I would prefer to keep the actual processing logic in the main otomo-rs crate, but the way rtic uses ISR's to create task dispatchers means that I will always have some hardware references in the main crate.

### debugging and logging
By default, the main crate uses defmt to log messages.  However, there is a full implementation of the log crate, and the logging "agent" can be configured through cargo features.
- default: `defmt_logger`
- serial: `serial_logger` (uses USART2, pins A2/A3)
- null: `null_logger`, no logging, but saves on space.

# TODO

- [X] get HC05 up and running
- [X] get communication running between phone and device
- [X] get protobuf messaging working
- [X] clean up init stuff <- lol that'll probably never gonna happen
- [X] joystick to tank conversion
- [ ] read ~~crappy~~ much better encoders (use the QEI interface)
- [ ] add PID library for speed control
- [ ] add ramp controller for fan current inrush
- [ ] read from IMU to get heading
- [ ] heading-based input to control loop
- [X] read from laser/ultrasonic/IR distance sensor
- [ ] obstacle avoidance: don't drive in direction of obstacle

# Optional TODO
- [X] log crate instead of defmt (configure through features)
- [ ] populate constants/thresholds at build time through config file
- [ ] messaging that doesn't obliterate heap usage and flash space
- [?] better separation of hardware control and logic
- [ ] LCD screen
- [ ] SD card logs
- [X] just...better robot chassis hardware.

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
- https://howtomechatronics.com/tutorials/arduino/arduino-and-hc-05-bluetooth-module-tutorial/
- https://robotics.stackexchange.com/questions/2011/how-to-calculate-the-right-and-left-speed-for-a-tank-like-rover
- https://en.wikipedia.org/wiki/KISS_(TNC)

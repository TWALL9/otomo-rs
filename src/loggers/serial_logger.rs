use crate::alloc::string::ToString;
use core::fmt::Write;

use log::{Level, LevelFilter, Metadata, Record};
use otomo_hardware::serial::DebugSerialPort;

pub type LoggerType = DebugSerialPort;

struct DummyType;

static DUMMY_LOGGER: DummyType = DummyType;
static mut SERIAL_LOGGER: Option<LoggerType> = None;

pub fn init(logger: LoggerType) {
    unsafe { SERIAL_LOGGER = Some(logger) }
    log::set_logger(&DUMMY_LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);
}

impl log::Log for DummyType {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let args = record.args();
            let record_str = if let Some(s) = args.as_str() {
                s.to_string()
            } else {
                args.to_string()
            };
            let level = match record.metadata().level() {
                Level::Trace => "trace",
                Level::Debug => "debug",
                Level::Info => "info",
                Level::Warn => "warn",
                Level::Error => "error",
            };

            unsafe {
                if let Some(tx) = &mut SERIAL_LOGGER {
                    // writeln!(tx, "{}: {}\r", level, record_str).unwrap();
                    writeln!(tx, "asdf").unwrap();
                }
            }
        }
    }

    fn flush(&self) {}
}

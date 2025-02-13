use crate::alloc::string::ToString;
use core::{fmt::Write, ptr::addr_of_mut};

use log::{Level, Metadata, Record};
use otomo_hardware::serial::DebugSerialPort;

pub type LoggerType = DebugSerialPort;

pub(crate) struct DummyType;

static DUMMY_LOGGER: DummyType = DummyType;
static mut SERIAL_LOGGER: Option<LoggerType> = None;
static mut LEVEL: Level = Level::Debug;

pub fn init(logger: LoggerType) {
    unsafe {
        SERIAL_LOGGER = Some(logger);
    }
}

pub(super) fn set_level(level: Level) {
    unsafe {
        LEVEL = level;
    }
}

pub(super) fn get_logger() -> &'static impl log::Log {
    &DUMMY_LOGGER
}

impl log::Log for DummyType {
    fn enabled(&self, metadata: &Metadata) -> bool {
        unsafe { metadata.level() <= LEVEL }
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
                Level::Trace => "[TRACE] ",
                Level::Debug => "[DEBUG] ",
                Level::Info => "[INFO] ",
                Level::Warn => "[WARN] ",
                Level::Error => "[ERROR] ",
            };

            unsafe {
                if let Some(tx) = &mut *addr_of_mut!(SERIAL_LOGGER) {
                    tx.write_str(level).unwrap();
                    tx.write_str(&record_str).unwrap();
                    tx.write_str("\r\n").unwrap();
                }
            }
        }
    }

    fn flush(&self) {}
}

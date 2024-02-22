use log::{Level, LevelFilter, Metadata, Record};

pub struct LoggerType;

static NULL_LOGGER: LoggerType = LoggerType;

pub fn init(_logger: LoggerType, level: Level) {
    log::set_logger(&NULL_LOGGER).unwrap();
    log::set_max_level(level);
}

impl log::Log for LoggerType {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, _record: &Record) {}

    fn flush(&self) {}
}

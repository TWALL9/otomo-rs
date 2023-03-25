use crate::alloc::string::ToString;

use defmt::{debug, error, info, trace, warn};
use defmt_rtt as _;
use log::{Level, LevelFilter, Metadata, Record};

pub struct LoggerType;

static DEFMT_LOGGER: LoggerType = LoggerType;

pub fn init(_logger: LoggerType) {
    log::set_logger(&DEFMT_LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);
}

impl log::Log for LoggerType {
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
            match record.metadata().level() {
                Level::Trace => trace!("{:?}", record_str.as_str()),
                Level::Debug => debug!("{:?}", record_str.as_str()),
                Level::Info => info!("{:?}", record_str.as_str()),
                Level::Warn => warn!("{:?}", record_str.as_str()),
                Level::Error => error!("{:?}", record_str.as_str()),
            }
        }
    }

    fn flush(&self) {}
}

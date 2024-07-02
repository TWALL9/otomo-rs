#[cfg(feature = "defmt_logger")]
pub mod defmt_logger;

#[cfg(feature = "null_logger")]
#[cfg(not(feature = "defmt_logger"))]
#[cfg(not(feature = "serial_logger"))]
pub mod null_logger;

#[cfg(feature = "serial_logger")]
pub mod serial_logger;

pub use log::Level;
use log::{Metadata, Record};

pub struct LoggerType;

static LOGGER: LoggerType = LoggerType;
static mut LEVEL: Level = Level::Debug;

pub fn init(_logger: LoggerType, level: log::Level) {
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(level.to_level_filter());
    unsafe {
        LEVEL = level;
    }

    #[cfg(feature = "defmt_logger")]
    defmt_logger::set_level(level);

    #[cfg(feature = "serial_logger")]
    serial_logger::set_level(level);
}

impl log::Log for LoggerType {
    fn enabled(&self, metadata: &Metadata) -> bool {
        unsafe { metadata.level() <= LEVEL }
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            #[cfg(feature = "defmt_logger")]
            unsafe {
                let logger = defmt_logger::get_logger();
                logger.log(record);
            }

            #[cfg(feature = "serial_logger")]
            unsafe {
                let logger = serial_logger::get_logger();
                logger.log(record);
            }
        }
    }

    fn flush(&self) {
        #[cfg(feature = "defmt_logger")]
        unsafe {
            let logger = defmt_logger::get_logger();
            logger.flush();
        }

        #[cfg(feature = "serial_logger")]
        unsafe {
            let logger = serial_logger::get_logger();
            logger.flush();
        }
    }
}

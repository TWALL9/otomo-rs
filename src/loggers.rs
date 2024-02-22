#[cfg(feature = "defmt_logger")]
#[cfg(not(feature = "null_logger"))]
#[cfg(not(feature = "serial_logger"))]
pub mod defmt_logger;

#[cfg(feature = "null_logger")]
#[cfg(not(feature = "defmt_logger"))]
#[cfg(not(feature = "serial_logger"))]
pub mod null_logger;

#[cfg(feature = "serial_logger")]
#[cfg(not(feature = "defmt_logger"))]
#[cfg(not(feature = "null_logger"))]
pub mod serial_logger;

pub use log::Level;

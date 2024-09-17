use alloc::vec::Vec;

use prost::{DecodeError, Message};

use log::Level;

include!(concat!(env!("OUT_DIR"), "/otomo.rs"));

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TopMsgTags {
    Config = 1,
    Joystick = 2,
    Fan = 3,
    State = 4,
    DriveResponse = 5,
    Pid = 6,
    Imu = 7,
    Battery = 8,
    DiffDrive = 10,
    Other = 0xFF,
}

impl TopMsg {
    pub fn which_msg(&self) -> TopMsgTags {
        match self.msg {
            Some(top_msg::Msg::Config(_)) => TopMsgTags::Config,
            Some(top_msg::Msg::Joystick(_)) => TopMsgTags::Joystick,
            Some(top_msg::Msg::DiffDrive(_)) => TopMsgTags::DiffDrive,
            Some(top_msg::Msg::Fan(_)) => TopMsgTags::Fan,
            Some(top_msg::Msg::State(_)) => TopMsgTags::State,
            Some(top_msg::Msg::DriveResponse(_)) => TopMsgTags::DriveResponse,
            Some(top_msg::Msg::Pid(_)) => TopMsgTags::Pid,
            Some(top_msg::Msg::Imu(_)) => TopMsgTags::Imu,
            Some(top_msg::Msg::Battery(_)) => TopMsgTags::Battery,
            None => TopMsgTags::Other,
        }
    }
}

impl Into<Level> for LogLevel {
    fn into(self) -> Level {
        match self {
            LogLevel::Trace => Level::Trace,
            LogLevel::Debug => Level::Debug,
            LogLevel::Info => Level::Info,
            LogLevel::Warn => Level::Warn,
            LogLevel::Error => Level::Error,
        }
    }
}

pub fn decode_proto_msg<M: Message + Default>(buf: &[u8]) -> Result<M, DecodeError> {
    M::decode(buf)
}

pub fn encode_proto<M: Message + Default>(msg: M) -> Result<Vec<u8>, &'static str> {
    let mut encoded = Vec::new();
    msg.encode(&mut encoded).map_err(|_| "encode err")?;
    Ok(encoded)
}

use alloc::vec::Vec;

use prost::{DecodeError, Message};

include!(concat!(env!("OUT_DIR"), "/otomo.rs"));

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TopMsgTags {
    Config = 1,
    Joystick = 2,
    DiffDrive = 10,
    Fan = 3,
    State = 4,
    DriveResponse = 5,
    Pid = 6,
    Log = 99,
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
            Some(top_msg::Msg::Log(_)) => TopMsgTags::Log,
            None => TopMsgTags::Other,
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

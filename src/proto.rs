use alloc::vec::Vec;

use prost::{DecodeError, Message};

include!(concat!(env!("OUT_DIR"), "/otomo.rs"));

pub fn decode_proto_msg<M: Message + Default>(buf: &[u8]) -> Result<M, DecodeError> {
    M::decode(buf)
}

pub fn encode_proto<M: Message + Default>(msg: M) -> Result<Vec<u8>, &'static str> {
    let mut encoded = Vec::new();
    msg.encode(&mut encoded).map_err(|_| "encode err")?;
    Ok(encoded)
}

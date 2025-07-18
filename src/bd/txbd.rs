//! Enhanced transmit buffer descriptor layout and fields.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

#[repr(C)]
pub struct TxBD {
    pub data_length: AtomicU16,
    pub flags: AtomicU16,
    pub data_buffer_pointer: AtomicU32,
    pub errors: AtomicU16,
    pub control: AtomicU16,
    pub launch_time: AtomicU32,
    _reserved0: [u16; 1],
    pub last_bdu: AtomicU16,
    pub timestamp_1588: AtomicU32,
    _reserved1: [u16; 4],
}

pub const FLAGS_READY: u16 = 1 << 15;
pub const FLAGS_WRAP: u16 = 1 << 13;
pub const FLAGS_LAST_IN: u16 = 1 << 11;
pub const FLAGS_TRANSMIT_CRC: u16 = 1 << 10;

impl TxBD {
    pub(crate) fn is_ready(&self) -> bool {
        self.flags.load(Ordering::SeqCst) & FLAGS_READY != 0
    }
}

#[cfg(test)]
mod tests {
    use super::TxBD;
    use std::ptr::addr_of;

    fn zeroed() -> TxBD {
        // Safety: zero bitpattern is fine for primitive fields.
        unsafe { core::mem::MaybeUninit::zeroed().assume_init() }
    }

    #[test]
    fn field_offsets() {
        let txbd = zeroed();
        let start = &txbd as *const _ as *const u8;
        assert_eq!(unsafe { start.add(0x0) }, addr_of!(txbd.data_length).cast());
        assert_eq!(unsafe { start.add(0x2) }, addr_of!(txbd.flags).cast());
        assert_eq!(
            unsafe { start.add(0x4) },
            addr_of!(txbd.data_buffer_pointer).cast()
        );
        assert_eq!(unsafe { start.add(0x8) }, addr_of!(txbd.errors).cast());
        assert_eq!(unsafe { start.add(0xA) }, addr_of!(txbd.control).cast());
        assert_eq!(unsafe { start.add(0xC) }, addr_of!(txbd.launch_time).cast());
        assert_eq!(unsafe { start.add(0x12) }, addr_of!(txbd.last_bdu).cast());
        assert_eq!(
            unsafe { start.add(0x14) },
            addr_of!(txbd.timestamp_1588).cast()
        );
    }
}

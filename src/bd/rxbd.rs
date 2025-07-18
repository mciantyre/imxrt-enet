//! Enhanced receive buffer descriptor layout and fields.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

#[repr(C)]
pub struct RxBD {
    pub data_length: AtomicU16,
    pub flags: AtomicU16,
    pub data_buffer_pointer: AtomicU32,
    pub status: AtomicU16,
    pub control: AtomicU16,
    pub checksum: AtomicU16,
    pub header: AtomicU16,
    _reserved0: [u16; 1],
    pub last_bdu: AtomicU16,
    pub timestamp_1588: AtomicU32,
    _reserved1: [u16; 4],
}

pub const FLAGS_EMPTY: u16 = 1 << 15;
pub const FLAGS_WRAP: u16 = 1 << 13;

impl RxBD {
    pub(crate) fn is_empty(&self) -> bool {
        self.flags.load(Ordering::SeqCst) & FLAGS_EMPTY != 0
    }
}

#[cfg(test)]
mod tests {
    use core::ptr::addr_of;

    use super::RxBD;

    fn zeroed() -> RxBD {
        // Safety: zero bitpattern is fine for primitive fields.
        unsafe { core::mem::MaybeUninit::zeroed().assume_init() }
    }

    #[test]
    fn field_offsets() {
        let rxbd = zeroed();
        let start = &rxbd as *const _ as *const u8;
        assert_eq!(unsafe { start.add(0x0) }, addr_of!(rxbd.data_length).cast());
        assert_eq!(unsafe { start.add(0x2) }, addr_of!(rxbd.flags).cast());
        assert_eq!(
            unsafe { start.add(0x4) },
            addr_of!(rxbd.data_buffer_pointer).cast()
        );
        assert_eq!(unsafe { start.add(0x8) }, addr_of!(rxbd.status).cast());
        assert_eq!(unsafe { start.add(0xA) }, addr_of!(rxbd.control).cast());
        assert_eq!(unsafe { start.add(0xC) }, addr_of!(rxbd.checksum).cast());
        assert_eq!(unsafe { start.add(0xE) }, addr_of!(rxbd.header).cast());
        assert_eq!(unsafe { start.add(0x12) }, addr_of!(rxbd.last_bdu).cast());
        assert_eq!(
            unsafe { start.add(0x14) },
            addr_of!(rxbd.timestamp_1588).cast()
        );
    }
}

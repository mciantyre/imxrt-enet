//! Enhanced receive buffer descriptor layout and fields.

use ral_registers::{RORegister, RWRegister};

#[repr(C)]
pub struct RxBD {
    pub data_length: RORegister<u16>,
    pub flags: RWRegister<u16>,
    pub data_buffer_pointer: RWRegister<u32>,
    pub status: RWRegister<u16>,
    pub control: RWRegister<u16>,
    pub checksum: RORegister<u16>,
    pub header: RORegister<u16>,
    _reserved0: [u16; 1],
    pub last_bdu: RWRegister<u16>,
    pub timestamp_1588: RORegister<u32>,
    _reserved1: [u16; 4],
}

bdfields!(flags, u16,
    empty               [ offset = 15, bits = 1, ],
    ro1                 [ offset = 14, bits = 1, ],
    wrap                [ offset = 13, bits = 1, ],
    ro2                 [ offset = 12, bits = 1, ],
    last                [ offset = 11, bits = 1, ],

    miss                [ offset =  8, bits = 1, ],
    broadcast           [ offset =  7, bits = 1, ],
    multicast           [ offset =  6, bits = 1, ],
    length_violation    [ offset =  5, bits = 1, ],
    non_octet_violation [ offset =  4, bits = 1, ],

    crc_error           [ offset =  2, bits = 1, ],
    overrun             [ offset =  1, bits = 1, ],
    truncated           [ offset =  0, bits = 1, ],
);

bdfields!(status, u16,
    vlan_priority           [ offset = 13, bits = 3, ],
    ip_checksum_error       [ offset =  5, bits = 1, ],
    protocol_checksum_error [ offset =  4, bits = 1, ],
    vlan                    [ offset =  2, bits = 1, ],
    ipv6                    [ offset =  1, bits = 1, ],
    frag                    [ offset =  0, bits = 1, ],
);

bdfields!(control, u16,
    mac_error               [ offset = 15, bits = 1, ],
    phy_error               [ offset = 10, bits = 1, ],
    collision               [ offset =  9, bits = 1, ],
    unicast                 [ offset =  8, bits = 1, ],
    interrupt               [ offset =  7, bits = 1, ],
);

bdfields!(header, u16,
    length      [ offset = 11, bits = 5, ],
    protocol    [ offset =  0, bits = 8, ],
);

bdfields!(last_bdu, u16,
    last_bdu    [ offset = 15, bits = 1, ],
);

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

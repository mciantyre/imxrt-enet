//! Enhanced transmit buffer descriptor layout and fields.

use ral_registers::RWRegister;

#[repr(C)]
pub struct TxBD {
    pub data_length: RWRegister<u16>,
    pub flags: RWRegister<u16>,
    pub data_buffer_pointer: RWRegister<u32>,
    pub errors: RWRegister<u16>,
    pub control: RWRegister<u16>,
    pub launch_time: RWRegister<u32>,
    _reserved0: [u16; 1],
    pub last_bdu: RWRegister<u16>,
    pub timestamp_1588: RWRegister<u32>,
    _reserved1: [u16; 4],
}

bdfields!(flags, u16,
    ready           [ offset = 15, bits = 1, ],
    to1             [ offset = 14, bits = 1, ],
    wrap            [ offset = 13, bits = 1, ],
    to2             [ offset = 12, bits = 1, ],
    last_in         [ offset = 11, bits = 1, ],
    transmit_crc    [ offset = 10, bits = 1, ],
);

bdfields!(errors, u16,
    transmit            [ offset = 15, bits = 1, ],
    underflow           [ offset = 13, bits = 1, ],
    excess_collision    [ offset = 12, bits = 1, ],
    frame_error         [ offset = 11, bits = 1, ],
    late_collision      [ offset = 10, bits = 1, ],
    overflow            [ offset =  9, bits = 1, ],
    timestamp           [ offset =  8, bits = 1, ],
);

bdfields!(control, u16,
    interrupt           [ offset = 14, bits = 1, ],
    timestamp           [ offset = 13, bits = 1, ],
    pins                [ offset = 12, bits = 1, ],
    iins                [ offset = 11, bits = 1, ],
    utlt                [ offset =  8, bits = 1, ],
    ftype               [ offset =  4, bits = 4, NON_AVB = 0, AVB_A = 1, AVB_B = 2 ],
);

bdfields!(last_bdu, u16,
    last_bdu            [ offset = 15, bits = 1, ],
);

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

    #[test]
    fn bdfields_enum() {
        let txbd = zeroed();
        ral_registers::modify_reg!(super, &txbd, control, ftype: AVB_B);
        assert_eq!(txbd.control.read(), 0x2 << 4);
        ral_registers::modify_reg!(super, &txbd, control, interrupt: 1);
        assert_eq!(txbd.control.read(), 0x2 << 4 | 1 << 14);
        ral_registers::modify_reg!(super, &txbd, control, ftype: 0);
        assert_eq!(txbd.control.read(), 1 << 14);
    }
}

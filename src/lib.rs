//! Ethernet driver for i.MX RT MCUs.

#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![deny(unsafe_op_in_unsafe_fn)]

mod bd;

pub use bd::{IoBuffers, IoSlices, ReceiveBuffers, ReceiveSlices, TransmitBuffers, TransmitSlices};
use imxrt_ral as ral;

pub use mdio::miim::{Read as MiimRead, Write as MiimWrite};
pub use smoltcp;

/// Allows independent transmit and receive functions.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum Duplex {
    /// Transmit and receive functions cannot overlap.
    ///
    /// Specifically, you cannot transmit frames while you're receiving frames.
    /// Similarly, you cannot receive frames while you're sending frames.
    Half,
    /// The MAC can transmit and receive simultaneously.
    ///
    /// Specifically, the receive path operates independent of the transmit
    /// path. You can transmit frames without concern for carrier sense and
    /// collision signals.
    Full,
}

/// Ethernet MAC and related functions.
///
/// The MDIO interface is always enabled. To generally use the MDIO interface,
/// use [`MiimRead`] and [`MiimWrite`]. Once your driver is configured, use
/// [`enable_mac`](Enet::enable_mac) to enable the transmit and receive datapaths.
///
/// The MAC implements the `phy` interfaces from [`smoltcp`]. The driver optimizes
/// for hardware-based checksumming as much as possible, but this only applies to
/// the network and transport layers.
pub struct Enet<const N: u8> {
    enet: ral::enet::Instance<N>,
    tx_ring: TransmitSlices<'static>,
    rx_ring: ReceiveSlices<'static>,
}

impl<const N: u8> Enet<N> {
    pub fn new(
        enet: ral::enet::Instance<N>,
        tx_ring: TransmitSlices<'static>,
        rx_ring: ReceiveSlices<'static>,
        source_clock_hz: u32,
        mac: &[u8; 6],
    ) -> Self {
        // Reset the module.
        ral::modify_reg!(ral::enet, enet, ECR, RESET: 1);

        ral::modify_reg!(ral::enet, enet, ECR,
            DBSWP: 1,   // Swap data for this little endian device.
            EN1588: 1,  // Use enhanced buffer descriptors.
            RESET: 0,   // I think this auto-clears, but just in case...
            DBGEN: 0,   // Keep running the MAC in debug mode.
        );

        // Turn off all interrupts.
        ral::write_reg!(ral::enet, enet, EIMR, 0);

        // The maximum receive buffer size includes four low bits of the register.
        // The user's buffer needs to be a non-zero multiple of 16 to account for
        // those extra bytes. We double-check this by asserting the requirement at
        // compile time in the IoBuffer types.
        debug_assert!(rx_ring.mtu() != 0 && rx_ring.mtu() & 0xF == 0);
        ral::write_reg!(ral::enet, enet, MRBR, R_BUF_SIZE: (rx_ring.mtu() >> 4) as u32);

        // Descriptor rings are pre-configured when the user acquires the slices.
        ral::write_reg!(ral::enet, enet, TDSR, tx_ring.as_ptr() as _);
        ral::write_reg!(ral::enet, enet, RDSR, rx_ring.as_ptr() as _);

        const SMI_MDC_FREQUENCY_HZ: u32 = 2_500_000;
        let mii_speed = source_clock_hz.div_ceil(2 * SMI_MDC_FREQUENCY_HZ) - 1;
        let hold_time = 10_u32.div_ceil(1_000_000_000 / source_clock_hz) - 1;
        // TODO no way to enable / disable the MII management frame preamble. Maybe a new method
        // for the user?
        ral::modify_reg!(ral::enet, enet, MSCR, HOLDTIME: hold_time, MII_SPEED: mii_speed);

        ral::modify_reg!(ral::enet, enet, RCR,
            // Default max frame length without VLAN tags.
            MAX_FL: 1518,
            // Since we're providing half-duplex control to the user, we
            // can't also enabled loopback.
            LOOP: 0,
            // No need to snoop.
            PROM: 0,
            // Do not reject broadcast frames; we might be interested
            // in these.
            BC_REJ: 0,
            // The MAC doesn't supply pause frames to the application.
            PAUFWD: 0,
            // Drop padding, along with the CRC, when supplying frames
            // to our software. This configuration implicitly includes
            // the CRC, so the CRCFWD below has no effect.
            PADEN: 1,
            // Drop the CRC in received frames. This doesn't turn off
            // CRC checking at the hardware level.
            //
            // If PADEN is set, this configuration does nothing.
            CRCFWD: 1,
            // Check the payload length based on the expected frame type /
            // frame length (encoded in the frame).
            NLC: 1,
            // Enable flow control; react to pause frames by pausing the data
            // transmit paths.
            FCE: 1,
            // MII or RMII mode; must be set.
            MII_MODE: 1,
            // Default to MII; users can enable RMII later.
            RMII_MODE: 0,
            // Default to 100Mbit/sec; users can throttle later.
            RMII_10T: 0,
        );

        ral::modify_reg!(ral::enet, enet, TCR,
            // smoltcp is not including frame CRCs from software. Let
            // the hardware handle it.
            CRCFWD: 0,
            // smoltcp is including the address in its frames, so we'll
            // pass it through.
            ADDINS: 0,
        );

        // Enable store-and-forward: start transmitting once you have a complete
        // frame in the FIFO.
        ral::modify_reg!(ral::enet, enet, TFWR, STRFWD: 1);
        // Maintain store-and-forward on the receive path: use the receive queue
        // as a buffer until an entire frame is received.
        ral::write_reg!(ral::enet, enet, RSFL, 0);

        // These accelerator options assume store-and-forward operations on both
        // data paths. See above.
        ral::modify_reg!(ral::enet, enet, RACC,
            // Discard frames with MAC errors (checksumming, length, PHY errors).
            LINEDIS: 1,
            // Discard frames with the wrong checksums for the protocol and headers.
            PRODIS: 1,
            IPDIS: 1,
            // Discard any padding within a short IP datagram.
            PADREM: 1,
        );
        ral::modify_reg!(ral::enet, enet, TACC,
            // Enable protocol checksums. Assumes that smoltcp sets these fields
            // to zero on our behalf.
            PROCHK: 1,
            // Enable IP checksum injection into the IPv4 header. Assumes that smoltcp
            // sets these fields to zero on our behalf.
            IPCHK: 1,
        );

        // Commit the MAC address so we can match against it in the receive path.
        ral::write_reg!(
            ral::enet,
            enet,
            PALR,
            (mac[0] as u32) << 24 | (mac[1] as u32) << 16 | (mac[2] as u32) << 8 | (mac[3] as u32)
        );
        ral::write_reg!(
            ral::enet,
            enet,
            PAUR,
            (mac[4] as u32) << 24 | (mac[5] as u32) << 16
        );

        Self {
            enet,
            tx_ring,
            rx_ring,
        }
    }

    /// Enable (`true`) or disable (`false`) the MAC.
    ///
    /// A disabled MAC cannot receive or send frames. By default, the MAC is disabled,
    /// and you'll need to enable it once you've completed driver configuration.
    #[inline]
    pub fn enable_mac(&mut self, enable: bool) {
        ral::modify_reg!(ral::enet, self.enet, ECR, ETHEREN: enable as u32);
        if enable {
            ral::write_reg!(ral::enet, self.enet, RDAR, RDAR: 1);
        }
    }

    /// Indicates if the ENET MAC is (`true`) or is not (`false`) enabled.
    #[inline]
    pub fn is_mac_enabled(&self) -> bool {
        ral::read_reg!(ral::enet, self.enet, ECR, ETHEREN == 1)
    }

    /// Enable (`true`) or disable (`false`) RMII mode.
    ///
    /// By default, the driver is in MII mode.
    ///
    /// # Panics
    ///
    /// Panics if called while the MAC is enabled.
    // TODO(mciantyre) enums for MII modes, speeds, duplex?
    #[inline]
    pub fn enable_rmii_mode(&mut self, enable: bool) {
        debug_assert!(!self.is_mac_enabled());
        ral::modify_reg!(ral::enet, self.enet, RCR, RMII_MODE: enable as u32);
    }

    /// Throttle the receive pathway to 10Mbit/s.
    ///
    /// When enabled, the recieve pathway operates in 10Mbit/s.
    /// By default, or when disabled, the receive pathway is at
    /// 100Mbit/s.
    ///
    /// # Panics
    ///
    /// Panics if called while the MAC is enabled.
    // TODO(mciantyre) enums for MII modes, speeds, duplex?
    #[inline]
    pub fn enable_10t_mode(&mut self, enable: bool) {
        debug_assert!(!self.is_mac_enabled());
        ral::modify_reg!(ral::enet, self.enet, RCR, RMII_10T: enable as u32);
    }

    /// Set the half-/full-duplex operation of the MAC.
    ///
    /// For more information, see the [`Duplex`] documentation.
    ///
    /// # Panics
    ///
    /// Panics if called while the MAC is enabled.
    #[inline]
    pub fn set_duplex(&mut self, duplex: Duplex) {
        debug_assert!(!self.is_mac_enabled());
        match duplex {
            Duplex::Full => {
                ral::modify_reg!(ral::enet, self.enet, TCR, FDEN: 1);
                ral::modify_reg!(ral::enet, self.enet, RCR, DRT: 0);
            }
            Duplex::Half => {
                ral::modify_reg!(ral::enet, self.enet, TCR, FDEN: 0);
                ral::modify_reg!(ral::enet, self.enet, RCR, DRT: 1);
            }
        }
    }

    /// Enable (`true`) or disable (`false`) management information database
    /// (MIB) statistic indicators.
    ///
    /// When enabled, the hardware tracks various types of errors in the
    /// MIB and remote network monitoring registers.
    #[inline]
    pub fn enable_mib(&mut self, enable: bool) {
        ral::modify_reg!(ral::enet, self.enet, MIBC, MIB_DIS: !enable as u32);
    }

    /// Set to zero all management information database (MIB) statistic indicators.
    #[inline]
    pub fn clear_mib(&mut self) {
        ral::modify_reg!(ral::enet, self.enet, MIBC, MIB_CLEAR: 1);
        ral::modify_reg!(ral::enet, self.enet, MIBC, MIB_CLEAR: 0);
    }
}

#[doc(hidden)]
pub struct TxReady<'a> {
    enet: &'a ral::enet::RegisterBlock,
}

impl TxReady<'_> {
    fn consume(self) {
        ral::write_reg!(ral::enet, self.enet, TDAR, TDAR: 1);
    }
}

#[doc(hidden)]
pub struct RxReady<'a> {
    enet: &'a ral::enet::RegisterBlock,
}

impl RxReady<'_> {
    fn consume(self) {
        ral::write_reg!(ral::enet, self.enet, RDAR, RDAR: 1);
    }
}

/// An error during an MII transfer.
///
/// TODO where are they?
#[non_exhaustive]
#[derive(Debug, defmt::Format)]
pub enum MiiError {}

impl<const N: u8> mdio::Read for Enet<N> {
    type Error = MiiError;

    #[inline]
    fn read(&mut self, ctrl_bits: u16) -> Result<u16, Self::Error> {
        // Place the control bits in to the high half-word of the register.
        let mmfr = (ctrl_bits as u32) << 16;
        ral::write_reg!(ral::enet, self.enet, MMFR, mmfr);

        while ral::read_reg!(ral::enet, self.enet, EIR, MII == 0) {}
        ral::write_reg!(ral::enet, self.enet, EIR, MII: 1);

        // Automatically discards control bits.
        Ok(ral::read_reg!(ral::enet, self.enet, MMFR, DATA) as u16)
    }
}

impl<const N: u8> mdio::Write for Enet<N> {
    type Error = MiiError;

    #[inline]
    fn write(&mut self, ctrl_bits: u16, data_bits: u16) -> Result<(), Self::Error> {
        // Place control bits into high half-word of register.
        let mmfr = (ctrl_bits as u32) << 16 | data_bits as u32;
        ral::write_reg!(ral::enet, self.enet, MMFR, mmfr);

        while ral::read_reg!(ral::enet, self.enet, EIR, MII == 0) {}
        ral::write_reg!(ral::enet, self.enet, EIR, MII: 1);

        Ok(())
    }
}

impl<const N: u8> smoltcp::phy::Device for Enet<N> {
    type RxToken<'a> = bd::RxToken<'a>;
    type TxToken<'a> = bd::TxToken<'a>;

    fn receive(
        &mut self,
        _: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let tx = self.tx_ring.next_token(TxReady { enet: &self.enet })?;
        let rx = self.rx_ring.next_token(RxReady { enet: &self.enet })?;
        Some((rx, tx))
    }

    fn transmit(&mut self, _: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        self.tx_ring.next_token(TxReady { enet: &self.enet })
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mtu = self.tx_ring.mtu().min(self.rx_ring.mtu());

        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.medium = smoltcp::phy::Medium::Ethernet;
        caps.max_transmission_unit = mtu;
        caps.max_burst_size = Some(mtu);

        caps.checksum.ipv4 = smoltcp::phy::Checksum::None;
        caps.checksum.udp = smoltcp::phy::Checksum::None;
        caps.checksum.tcp = smoltcp::phy::Checksum::None;
        caps.checksum.icmpv4 = smoltcp::phy::Checksum::None;

        caps
    }
}

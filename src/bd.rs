//! Enhanced buffer descriptors.
//!
//! Buffer descriptors (BD) are defined with register access layer (RAL) compatibility.
//! These definitions come from the i.MX RT 1170 reference manual, revision 2.

#![expect(
    clippy::new_without_default,
    reason = "default() isn't const, useless for static init"
)]

pub(crate) mod rxbd;
pub(crate) mod txbd;

use core::sync::atomic::Ordering;

#[repr(align(64))]
struct DescriptorRing<D, const N: usize>([D; N]);

#[repr(align(64))]
struct DataBuffer<const N: usize>([u8; N]);

pub struct IoBuffers<D, const COUNT: usize, const MTU: usize> {
    ring: DescriptorRing<D, COUNT>,
    buffers: [DataBuffer<MTU>; COUNT],
}

pub type TransmitBuffers<const COUNT: usize, const MTU: usize> = IoBuffers<txbd::TxBD, COUNT, MTU>;
pub type ReceiveBuffers<const COUNT: usize, const MTU: usize> = IoBuffers<rxbd::RxBD, COUNT, MTU>;

impl<D, const COUNT: usize, const MTU: usize> IoBuffers<D, COUNT, MTU> {
    const fn with_ring(ring: DescriptorRing<D, COUNT>) -> Self {
        assert!(MTU.is_multiple_of(16));
        Self {
            ring,
            buffers: [const { DataBuffer([0; MTU]) }; COUNT],
        }
    }

    fn init(
        &'static mut self,
        init_descriptors: impl Fn(&mut [D], &mut [DataBuffer<MTU>]),
    ) -> IoSlices<'static, D> {
        let ring = &mut self.ring.0;
        let buffers = self.buffers.as_mut_slice();
        init_descriptors(ring, buffers);
        IoSlices::new(ring, MTU)
    }
}

impl<const COUNT: usize, const MTU: usize> IoBuffers<txbd::TxBD, COUNT, MTU> {
    pub const fn new() -> Self {
        Self::with_ring(DescriptorRing([const { txbd::TxBD::zero() }; COUNT]))
    }

    pub fn take(&'static mut self) -> IoSlices<'static, txbd::TxBD> {
        self.init(|descriptors, buffers| {
            for (descriptor, buffer) in descriptors.iter_mut().zip(buffers.iter_mut()) {
                descriptor
                    .data_buffer_pointer
                    .store(buffer.0.as_mut_ptr() as _, Ordering::SeqCst);
            }

            // When the DMA engine reaches this descriptor, it needs to wrap
            // around to the first descriptor.
            if let Some(descriptor) = descriptors.last_mut() {
                descriptor
                    .flags
                    .fetch_or(txbd::FLAGS_WRAP, Ordering::SeqCst);
            }
        })
    }
}

impl<const COUNT: usize, const MTU: usize> IoBuffers<rxbd::RxBD, COUNT, MTU> {
    pub const fn new() -> Self {
        Self::with_ring(DescriptorRing([const { rxbd::RxBD::zero() }; COUNT]))
    }

    pub fn take(&'static mut self) -> IoSlices<'static, rxbd::RxBD> {
        self.init(|descriptors, buffers| {
            for (descriptor, buffer) in descriptors.iter_mut().zip(buffers.iter_mut()) {
                descriptor
                    .data_buffer_pointer
                    .store(buffer.0.as_mut_ptr() as _, Ordering::Relaxed);
                // Zero all other flags.
                descriptor.flags.store(rxbd::FLAGS_EMPTY, Ordering::SeqCst);
            }

            // When the DMA engine reaches this descriptor, it needs to wrap
            // around to the first descriptor.
            if let Some(descriptor) = descriptors.last_mut() {
                descriptor
                    .flags
                    .fetch_or(rxbd::FLAGS_WRAP, Ordering::SeqCst);
            }
        })
    }
}

pub struct IoSlices<'a, D> {
    ring: &'a mut [D],
    mtu: usize,
    index: usize,
}

pub type ReceiveSlices<'a> = IoSlices<'a, rxbd::RxBD>;
pub type TransmitSlices<'a> = IoSlices<'a, txbd::TxBD>;

impl<'a, D> IoSlices<'a, D> {
    fn new(ring: &'a mut [D], mtu: usize) -> Self {
        Self {
            ring,
            mtu,
            index: 0,
        }
    }
    pub(crate) fn as_ptr(&self) -> *const D {
        self.ring.as_ptr()
    }
    pub(crate) fn mtu(&self) -> usize {
        self.mtu
    }
    pub(crate) fn len(&self) -> usize {
        self.ring.len()
    }
}

impl<D> IoSlices<'_, D> {
    fn next_impl<'a, R: 'a>(
        &'a mut self,
        check: impl FnOnce(&D) -> bool,
        ready: R,
    ) -> Option<IoToken<'a, D, R>> {
        let next = (self.index + 1) % self.ring.len();
        let descriptor = self.ring.get_mut(self.index).unwrap();
        if check(descriptor) {
            Some(IoToken {
                descriptor,
                index: &mut self.index,
                next,
                mtu: self.mtu,
                ready,
            })
        } else {
            None
        }
    }
}

pub struct IoToken<'a, D, R> {
    descriptor: &'a mut D,
    index: &'a mut usize,
    next: usize,
    mtu: usize,
    ready: R,
}

pub type TxToken<'a> = IoToken<'a, txbd::TxBD, crate::TxReady<'a>>;
pub type RxToken<'a> = IoToken<'a, rxbd::RxBD, crate::RxReady<'a>>;

impl ReceiveSlices<'_> {
    pub(crate) fn next_token<'a>(&'a mut self, ready: crate::RxReady<'a>) -> Option<RxToken<'a>> {
        self.next_impl(|rxbd| !rxbd.is_empty(), ready)
    }
}

impl TransmitSlices<'_> {
    pub(crate) fn next_token<'a>(&'a mut self, ready: crate::TxReady<'a>) -> Option<TxToken<'a>> {
        self.next_impl(|txbd| !txbd.is_ready(), ready)
    }
}

impl smoltcp::phy::TxToken for TxToken<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // Safety: we ensure that smoltcp isn't exceeding the size of the buffer.
        // We know that the pointer is valid. Module inspection reveals that this is the
        // only mutable reference to the pointer; it's tracked through the descriptor
        // lifetimes.
        let buffer = unsafe {
            assert!(len <= self.mtu);
            let ptr = self.descriptor.data_buffer_pointer.load(Ordering::Relaxed) as *mut u8;
            core::slice::from_raw_parts_mut(ptr, len)
        };

        let result = f(buffer);

        self.descriptor
            .data_length
            .store(len as _, Ordering::Relaxed);
        self.descriptor.flags.fetch_or(
            txbd::FLAGS_READY | txbd::FLAGS_LAST_IN | txbd::FLAGS_TRANSMIT_CRC,
            Ordering::SeqCst,
        );
        self.ready.consume();
        *self.index = self.next;
        result
    }
}

impl smoltcp::phy::RxToken for RxToken<'_> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        // Safety: hardware will not exceed our maximum frame length. We know that
        // the pointer is valid; see discussion above.
        let buffer = unsafe {
            let len = self.descriptor.data_length.load(Ordering::Relaxed) as usize;
            assert!(len <= self.mtu);
            let ptr = self.descriptor.data_buffer_pointer.load(Ordering::Relaxed) as *mut u8;
            core::slice::from_raw_parts_mut(ptr, len)
        };

        let result = f(buffer);
        self.descriptor
            .flags
            .fetch_or(rxbd::FLAGS_EMPTY, Ordering::SeqCst);
        self.ready.consume();
        *self.index = self.next;
        result
    }
}

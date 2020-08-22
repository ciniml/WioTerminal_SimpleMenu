#![no_std]

use embedded_hal::spi::{FullDuplex};
use core::pin::Pin;
use embedded_dma::*;
use atsamd_hal::spi_common::CommonSpi;
use atsamd_hal::clock::GenericClockController;
use atsamd51p19a::{DMAC, MCLK, dmac::channel::chctrla::*};
use core::{convert::TryFrom, convert::TryInto, cell::RefCell};

#[derive(Debug, Copy, Clone)]
pub enum ESPTransportError {
    Busy,
}
#[derive(Debug, Copy, Clone)]
pub enum OperationStatus {
    Running,
    Completed,
}
#[derive(Debug, Copy, Clone)]
pub enum AsyncOperationError {
    NotSupported,
    Busy,
}

pub trait AsyncOperation<T> {
    fn poll(self: &Self) -> OperationStatus;
    fn get_result(self: Self) -> T;
    fn cancel(self: &mut Self) -> Result<(), AsyncOperationError>;
}

pub struct Transfer<SPI: CommonSpi, Buffer> {
    buffer: Buffer,
    transport: ESPTransport<SPI>,
}

pub struct TransferResult<SPI: CommonSpi, Buffer> {
    transport: ESPTransport<SPI>,
    buffer: Buffer,
    length: usize,
}

impl <SPI: CommonSpi, Buffer> AsyncOperation<TransferResult<SPI, Buffer>> for Transfer<SPI, Buffer> {
    fn poll(self: &Self) -> OperationStatus {
        todo!()
    }
    fn get_result(self: Self) -> TransferResult<SPI, Buffer> {
        todo!()
    }
    fn cancel(self: &mut Self) -> Result<(), AsyncOperationError> {
        todo!()
    }
   
}

pub struct ESPTransport<SPI: CommonSpi> {
    spi: SPI,
}

impl <SPI: CommonSpi> ESPTransport<SPI> {
    fn new(spi: SPI) -> Self {
        Self {
            spi: spi,
        }
    }
    fn begin_read<'a, Buffer: WriteBuffer>(self: Self, buffer: &'a mut Buffer) -> Transfer<SPI, &'a mut Buffer> {
        return Transfer {
            buffer: buffer,
            transport: self,
        };
    }
    fn begin_write<'a, Data: ReadBuffer>(self: Self, data: &'a Data) -> Transfer<SPI, &'a Data> {
        todo!()
    }
}

pub struct Dmac {
    dmac: RefCell<DMAC>,
    descriptors: RefCell<[DmaDescriptor; 32]>,
    write_backs: RefCell<[DmaDescriptor; 32]>,
}

pub struct DmacChannel<'dmac> {
    index: u8,
    dmac: &'dmac RefCell<DMAC>,
    descriptors: &'dmac RefCell<[DmaDescriptor; 32]>,
}

pub struct DmacTransfer<'dmac, Buffer> {
    channel: DmacChannel<'dmac>,
    buffer: Buffer,
}

pub struct DmacMemTransfer<Channel, SourceBuffer, DestinationBuffer> {
    channel: Channel,
    source: SourceBuffer,
    destination: DestinationBuffer,
}

impl<Channel: DmacChannelTrait, SourceBuffer, DestinationBuffer> AsyncOperation<Channel> for DmacMemTransfer<Channel, SourceBuffer, DestinationBuffer> {
    fn poll(self: &Self) -> OperationStatus {
        if self.channel.is_transfer_in_progress()  {
            OperationStatus::Running
        } else {
            OperationStatus::Completed
        }
    }
    fn get_result(self: Self) -> Channel {
        while self.channel.is_transfer_in_progress() {}
        self.channel
    }
    fn cancel(self: &mut Self) -> Result<(), AsyncOperationError> {
        Err(AsyncOperationError::NotSupported)
    }

}

#[derive(Debug, Copy, Clone)]
pub enum DmacError {
    NotSupported,
    CounterTooLong,
}

impl Dmac {
    pub fn new(dmac: DMAC, mclk: &mut MCLK) -> Dmac {
        // Enable DMAC clock.
        mclk.ahbmask.modify(|_, w| { w.dmac_().set_bit()});
        // Reset the DMAC
        dmac.ctrl.write(|w| { w.swrst().set_bit() });
        while dmac.ctrl.read().swrst().bit_is_set() {}
        // Enable all priority levels.
        dmac.ctrl.modify(|_, w| { w.lvlen0().set_bit().lvlen1().set_bit().lvlen2().set_bit().lvlen3().set_bit() });
        // clear base adresses.
        dmac.baseaddr.reset();
        dmac.wrbaddr.reset();

        unsafe {
            Self {
                dmac: RefCell::new(dmac),
                descriptors: core::mem::zeroed(),
                write_backs: core::mem::zeroed(),
            }
        }
    }

    fn ensure_dma_enabled(self: &Self) {
        let dmac = self.dmac.borrow_mut();
        if dmac.ctrl.read().dmaenable().bit_is_clear() {
            if dmac.baseaddr.read().baseaddr().bits() == 0  {
                let descriptors = self.descriptors.as_ptr();
                dmac.baseaddr.write(|w| unsafe { w.baseaddr().bits(descriptors as u32) })
            }
            if dmac.wrbaddr.read().wrbaddr().bits() == 0  {
                let descriptors = self.write_backs.as_ptr();
                dmac.wrbaddr.write(|w| unsafe { w.wrbaddr().bits(descriptors as u32) })
            }
            dmac.ctrl.modify(|_, w| { w.dmaenable().set_bit() });
        }
    }

    pub fn allocate_channel(self: &Self, index: u8) -> DmacChannel {
        DmacChannel::new(self, 0)
    }
}


trait DmacChannelTrait {
    fn is_transfer_in_progress(self: &Self) -> bool;
    fn has_transfer_error(self: &Self) -> bool;
}

impl<'dmac> DmacChannelTrait for DmacChannel<'dmac>  {
    fn is_transfer_in_progress(self: &Self) -> bool {
        let dmac_reg = self.dmac.borrow();
        dmac_reg.channel[self.index as usize].chstatus.read().busy().bit_is_set()
    }
    fn has_transfer_error(self: &Self) -> bool {
        let status = self.dmac.borrow().channel[self.index as usize].chstatus.read();
        status.crcerr().bit_is_set() || status.ferr().bit_is_set()
    }
}
impl<'dmac> DmacChannel<'dmac> {
    pub fn new(dmac: &'dmac Dmac, index: u8) -> DmacChannel<'dmac> {
        // Reset the channel
        dmac.dmac.borrow_mut().channel[index as usize].chctrla.modify(|_, w| { w.swrst().set_bit() });
        while dmac.dmac.borrow().channel[index as usize].chctrla.read().swrst().bit_is_set() {}
        // Ensure that DMA is enabled.
        dmac.ensure_dma_enabled();

        DmacChannel {
            dmac: &dmac.dmac,
            descriptors: &dmac.descriptors,
            index: index,
        }
    }

    
    pub fn transfer_mem<Word: WordType, Source: ReadBuffer<Word=Word>, Destination: WriteBuffer<Word=Word>>(self: Self, source: Source, mut destination: Destination, count: u16) -> Result<DmacMemTransfer<Self, Source, Destination>, DmacError> {
        let mut descriptors = self.descriptors.borrow_mut();
        let descriptor = descriptors.get_mut(self.index as usize).unwrap();
        unsafe {
            let read_buffer = source.read_buffer();
            let write_buffer = destination.write_buffer();
            if read_buffer.1 < count as usize || write_buffer.1 < count as usize  {
                return Err(DmacError::CounterTooLong);
            }
            descriptor.set_srcaddr(read_buffer.0, count);
            descriptor.set_dstaddr(write_buffer.0, count);
            descriptor.set_descaddr(None);
            descriptor.btcnt = count as u16;
            descriptor.btctrl.set_evosel(EVOSEL::BLOCK)
                             .set_blockact(BLOCKACT::NOACT)
                             .set_beatsize(Word::beatsize())
                             .set_srcinc()
                             .set_dstinc()
                             .set_stepsel(STEPSEL::DST)
                             .set_stepsize(0)
                             .set_valid();
            self.dmac.borrow_mut().channel[self.index as usize].chctrla.write(|w| { w.swrst().set_bit() });
            while self.dmac.borrow().channel[self.index as usize].chctrla.read().swrst().bit_is_set() {}
            
            let dmac = self.dmac.borrow_mut();
            dmac.channel[self.index as usize].chctrla.write(|w| {
                w   .trigsrc().variant(TRIGSRC_A::DISABLE)
                    .trigact().variant(TRIGACT_A::TRANSACTION)
                    .burstlen().variant(BURSTLEN_A::SINGLE)
                    .threshold().variant(THRESHOLD_A::_1BEAT)
                    .enable().set_bit()
            });
            dmac.swtrigctrl.write(|w| unsafe { w.bits(1u32 << self.index)});
        }
        Ok(DmacMemTransfer {
            channel: self,
            source: source,
            destination: destination,
        })
    }
}

#[repr(C)]
pub struct DmaDescriptor {
    btctrl: BTCTRL,
    btcnt: u16,
    srcaddr: u32,
    dstaddr: u32,
    descaddr: u32,
}

impl DmaDescriptor {
    fn new() -> Self {
        unsafe{
            Self {
                btctrl: core::mem::zeroed(),
                btcnt: 0,
                srcaddr: 0,
                dstaddr: 0,
                descaddr: 0,
            }
        }
    }

    unsafe fn set_srcaddr<Word: WordType>(self: &mut Self, buffer: *const Word, transfer_count: u16) -> &mut Self {
        self.srcaddr = (buffer as u32) + ((transfer_count as usize) * Word::word_size()) as u32;
        self
    }
    unsafe fn set_dstaddr<Word: WordType>(self: &mut Self, buffer: *const Word, transfer_count: u16) -> &mut Self {
        self.dstaddr = (buffer as u32) + ((transfer_count as usize) * Word::word_size()) as u32;
        self
    }
    unsafe fn set_descaddr(self: &mut Self, next_desc: Option<*const Self>) -> &mut Self {
        if let Some(p) = next_desc {
            self.descaddr = p as u32
        } else {
            self.descaddr = 0
        }
        self
    }
}

#[repr(C)]
pub struct BTCTRL {
    value: u16,
}

#[repr(u16)]
pub enum EVOSEL {
    DISABLE,
    BLOCK,
    BEAT = 3,
}

impl TryFrom<u16> for EVOSEL {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            value if value == EVOSEL::DISABLE as u16 => Ok(EVOSEL::DISABLE),
            value if value == EVOSEL::BLOCK as u16 => Ok(EVOSEL::BLOCK),
            value if value == EVOSEL::BEAT as u16 => Ok(EVOSEL::BEAT),
            _ => Err(()),
        }
    }
    
}

#[repr(u16)]
pub enum BLOCKACT {
    NOACT,
    INT,
    SUSPEND,
    BOTH,
}

impl TryFrom<u16> for BLOCKACT {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            value if value == BLOCKACT::NOACT as u16 => Ok(BLOCKACT::NOACT),
            value if value == BLOCKACT::INT as u16 => Ok(BLOCKACT::INT),
            value if value == BLOCKACT::SUSPEND as u16 => Ok(BLOCKACT::SUSPEND),
            value if value == BLOCKACT::BOTH as u16 => Ok(BLOCKACT::BOTH),
            _ => Err(()),
        }
    }
}

#[repr(u16)]
pub enum BEATSIZE {
    BYTE,
    HWORD,
    WORD,
}

trait WordType {
    fn beatsize() -> BEATSIZE;
    fn word_size() -> usize;
}
impl WordType for u8 {
    #[inline(always)]
    fn word_size() -> usize { 1 }
    #[inline(always)]
    fn beatsize() -> BEATSIZE { BEATSIZE::BYTE }
}
impl WordType for u16 {
    #[inline(always)]
    fn word_size() -> usize { 2 }
    #[inline(always)]
    fn beatsize() -> BEATSIZE { BEATSIZE::HWORD }
}
impl WordType for u32 {
    #[inline(always)]
    fn word_size() -> usize { 4 }
    #[inline(always)]
    fn beatsize() -> BEATSIZE { BEATSIZE::WORD }
}

impl TryFrom<u16> for BEATSIZE {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            value if value == BEATSIZE::BYTE as u16 => Ok(BEATSIZE::BYTE),
            value if value == BEATSIZE::HWORD as u16 => Ok(BEATSIZE::HWORD),
            value if value == BEATSIZE::WORD as u16 => Ok(BEATSIZE::WORD),
            _ => Err(()),
        }
    }
}

#[repr(u16)]
pub enum STEPSEL {
    DST,
    SRC,
}

impl TryFrom<u16> for STEPSEL {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            value if value == STEPSEL::DST as u16 => Ok(STEPSEL::DST),
            value if value == STEPSEL::SRC as u16 => Ok(STEPSEL::SRC),
            _ => Err(()),
        }
    }
}

macro_rules! define_field {
    ($getter_name:ident, $setter_name:ident: ($field_type:ty, $bit:expr, $width:expr)) => {
        fn $getter_name(self: &Self) -> $field_type { ((self.value >> $bit) & ((1u16 << $bit) - 1u16)).try_into().unwrap() }
        fn $setter_name(self: &mut Self, value: $field_type) -> &mut Self { self.value = (self.value & !(((1u16 << $bit) - 1u16) << $bit)) | (((value as u16) & ((1u16 << $bit) - 1u16)) << $bit); self }
    };
    ($is_name:ident, $set_name:ident, $clear_name:ident: ($bit:expr)) => {
        fn $is_name(self: &Self) -> bool { (self.value & (1u16 << $bit)) != 0 }
        fn $set_name(self: &mut Self) -> &mut Self { self.value |= (1u16 << $bit); self }
        fn $clear_name(self: &mut Self) -> &mut Self { self.value &= !(1u16 << $bit); self }
    };
    
}

impl BTCTRL {
    define_field!(is_valid, set_valid, clear_valid: (0));
    define_field!(get_evosel, set_evosel: (EVOSEL, 1, 2));
    define_field!(get_blockact, set_blockact: (BLOCKACT, 3, 2));
    define_field!(get_beatsize, set_beatsize: (BEATSIZE, 8, 2));
    define_field!(is_srcinc, set_srcinc, clear_srcinc: (10));
    define_field!(is_dstinc, set_dstinc, clear_dstinc: (11));
    define_field!(get_stepsel, set_stepsel: (STEPSEL, 12, 1));
    define_field!(get_stepsize, set_stepsize: (u16, 13, 3));
}
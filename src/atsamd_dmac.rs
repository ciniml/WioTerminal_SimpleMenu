#![no_std]

use embedded_hal::spi::FullDuplex;

use atsamd_hal::spi_common::CommonSpi;
use embedded_dma::*;

use atsamd51p19a::{dmac::channel::chctrla::*, DMAC, MCLK};
use core::{cell::RefCell, convert::TryFrom, convert::TryInto};

use crate::async_operation::*;

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

pub struct DmacTransfer<Channel, Buffer> {
    channel: Channel,
    buffer: Buffer,
}

pub struct DmacMemTransfer<Channel, SourceBuffer, DestinationBuffer> {
    channel: Channel,
    source: SourceBuffer,
    destination: DestinationBuffer,
}

impl<Channel: DmacChannelTrait, SourceBuffer, DestinationBuffer> AsyncOperation<Channel>
    for DmacMemTransfer<Channel, SourceBuffer, DestinationBuffer>
{
    fn poll(self: &Self) -> OperationStatus {
        if self.channel.is_transfer_in_progress() {
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
        mclk.ahbmask.modify(|_, w| w.dmac_().set_bit());
        // Reset the DMAC
        dmac.ctrl.write(|w| w.swrst().set_bit());
        while dmac.ctrl.read().swrst().bit_is_set() {}
        // Enable all priority levels.
        dmac.ctrl.modify(|_, w| {
            w.lvlen0()
                .set_bit()
                .lvlen1()
                .set_bit()
                .lvlen2()
                .set_bit()
                .lvlen3()
                .set_bit()
        });
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
            if dmac.baseaddr.read().baseaddr().bits() == 0 {
                let descriptors = self.descriptors.as_ptr();
                dmac.baseaddr
                    .write(|w| unsafe { w.baseaddr().bits(descriptors as u32) })
            }
            if dmac.wrbaddr.read().wrbaddr().bits() == 0 {
                let descriptors = self.write_backs.as_ptr();
                dmac.wrbaddr
                    .write(|w| unsafe { w.wrbaddr().bits(descriptors as u32) })
            }
            dmac.ctrl.modify(|_, w| w.dmaenable().set_bit());
        }
    }

    pub fn allocate_channel(self: &Self, _index: u8) -> DmacChannel {
        DmacChannel::new(self, 0)
    }
}

trait DmacChannelTrait {
    fn is_transfer_in_progress(self: &Self) -> bool;
    fn has_transfer_error(self: &Self) -> bool;
}

impl<'dmac> DmacChannelTrait for DmacChannel<'dmac> {
    fn is_transfer_in_progress(self: &Self) -> bool {
        let dmac_reg = self.dmac.borrow();
        dmac_reg.channel[self.index as usize]
            .chstatus
            .read()
            .busy()
            .bit_is_set()
    }
    fn has_transfer_error(self: &Self) -> bool {
        let status = self.dmac.borrow().channel[self.index as usize]
            .chstatus
            .read();
        status.crcerr().bit_is_set() || status.ferr().bit_is_set()
    }
}
impl<'dmac> DmacChannel<'dmac> {
    pub fn new(dmac: &'dmac Dmac, index: u8) -> DmacChannel<'dmac> {
        // Reset the channel
        dmac.dmac.borrow_mut().channel[index as usize]
            .chctrla
            .modify(|_, w| w.swrst().set_bit());
        while dmac.dmac.borrow().channel[index as usize]
            .chctrla
            .read()
            .swrst()
            .bit_is_set()
        {}
        // Ensure that DMA is enabled.
        dmac.ensure_dma_enabled();

        DmacChannel {
            dmac: &dmac.dmac,
            descriptors: &dmac.descriptors,
            index: index,
        }
    }

    pub fn transfer_mem<
        Word: WordType,
        Source: ReadBuffer<Word = Word>,
        Destination: WriteBuffer<Word = Word>,
    >(
        self: Self,
        source: Source,
        mut destination: Destination,
        count: u16,
    ) -> Result<DmacMemTransfer<Self, Source, Destination>, DmacError> {
        let mut descriptors = self.descriptors.borrow_mut();
        let descriptor = descriptors.get_mut(self.index as usize).unwrap();
        unsafe {
            let read_buffer = source.read_buffer();
            let write_buffer = destination.write_buffer();
            if read_buffer.1 < count as usize || write_buffer.1 < count as usize {
                return Err(DmacError::CounterTooLong);
            }
            descriptor.set_srcaddr(read_buffer.0, count);
            descriptor.set_dstaddr(write_buffer.0, count);
            descriptor.set_descaddr(None);
            descriptor.btcnt = count as u16;
            descriptor
                .btctrl
                .set_evosel(EVOSEL::BLOCK)
                .set_blockact(BLOCKACT::NOACT)
                .set_beatsize(Word::beatsize())
                .set_srcinc()
                .set_dstinc()
                .set_stepsel(STEPSEL::DST)
                .set_stepsize(0)
                .set_valid();
            self.dmac.borrow_mut().channel[self.index as usize]
                .chctrla
                .write(|w| w.swrst().set_bit());
            while self.dmac.borrow().channel[self.index as usize]
                .chctrla
                .read()
                .swrst()
                .bit_is_set()
            {}

            let dmac = self.dmac.borrow_mut();
            dmac.channel[self.index as usize].chctrla.write(|w| {
                w.trigsrc()
                    .variant(TRIGSRC_A::DISABLE)
                    .trigact()
                    .variant(TRIGACT_A::TRANSACTION)
                    .burstlen()
                    .variant(BURSTLEN_A::SINGLE)
                    .threshold()
                    .variant(THRESHOLD_A::_1BEAT)
                    .enable()
                    .set_bit()
            });
            dmac.swtrigctrl
                .write(|w| unsafe { w.bits(1u32 << self.index) });
        }
        Ok(DmacMemTransfer {
            channel: self,
            source: source,
            destination: destination,
        })
    }

    pub unsafe fn read_peripheral<Word: WordType, Destination: WriteBuffer<Word = Word>>(
        self: Self,
        source: *const Word,
        trigger: TRIGSRC,
        mut destination: Destination,
        count: u16,
    ) -> Result<DmacTransfer<Self, Destination>, DmacError> {
        let mut descriptors = self.descriptors.borrow_mut();
        let descriptor = descriptors.get_mut(self.index as usize).unwrap();
        unsafe {
            let write_buffer = destination.write_buffer();
            if write_buffer.1 < count as usize {
                return Err(DmacError::CounterTooLong);
            }
            descriptor.set_srcaddr(source, count);
            descriptor.set_dstaddr(write_buffer.0, count);
            descriptor.set_descaddr(None);
            descriptor.btcnt = count as u16;
            descriptor
                .btctrl
                .set_evosel(EVOSEL::BLOCK)
                .set_blockact(BLOCKACT::NOACT)
                .set_beatsize(Word::beatsize())
                .clear_srcinc()
                .set_dstinc()
                .set_stepsel(STEPSEL::DST)
                .set_stepsize(0)
                .set_valid();
            self.dmac.borrow_mut().channel[self.index as usize]
                .chctrla
                .write(|w| w.swrst().set_bit());
            while self.dmac.borrow().channel[self.index as usize]
                .chctrla
                .read()
                .swrst()
                .bit_is_set()
            {}

            let dmac = self.dmac.borrow_mut();
            dmac.channel[self.index as usize].chctrla.write(|w| {
                w.trigsrc()
                    .bits(trigger as u8)
                    .trigact()
                    .variant(TRIGACT_A::BURST)
                    .burstlen()
                    .variant(BURSTLEN_A::SINGLE)
                    .threshold()
                    .variant(THRESHOLD_A::_1BEAT)
                    .enable()
                    .set_bit()
            });
        }
        Ok(DmacTransfer {
            channel: self,
            buffer: destination,
        })
    }

    pub unsafe fn write_peripheral<Word: WordType, Source: ReadBuffer<Word = Word>>(
        self: Self,
        source: Source,
        destination: *const Word,
        trigger: TRIGSRC,
        count: u16,
    ) -> Result<DmacTransfer<Self, Source>, DmacError> {
        let mut descriptors = self.descriptors.borrow_mut();
        let descriptor = descriptors.get_mut(self.index as usize).unwrap();
        unsafe {
            let read_buffer = source.read_buffer();
            if read_buffer.1 < count as usize {
                return Err(DmacError::CounterTooLong);
            }
            descriptor.set_srcaddr(read_buffer.0, count);
            descriptor.set_dstaddr(destination, count);
            descriptor.set_descaddr(None);
            descriptor.btcnt = count as u16;
            descriptor
                .btctrl
                .set_evosel(EVOSEL::BLOCK)
                .set_blockact(BLOCKACT::NOACT)
                .set_beatsize(Word::beatsize())
                .set_srcinc()
                .clear_dstinc()
                .set_stepsel(STEPSEL::SRC)
                .set_stepsize(0)
                .set_valid();
            self.dmac.borrow_mut().channel[self.index as usize]
                .chctrla
                .write(|w| w.swrst().set_bit());
            while self.dmac.borrow().channel[self.index as usize]
                .chctrla
                .read()
                .swrst()
                .bit_is_set()
            {}

            let dmac = self.dmac.borrow_mut();
            dmac.channel[self.index as usize].chctrla.write(|w| {
                w.trigsrc()
                    .bits(trigger as u8)
                    .trigact()
                    .variant(TRIGACT_A::BURST)
                    .burstlen()
                    .variant(BURSTLEN_A::SINGLE)
                    .threshold()
                    .variant(THRESHOLD_A::_1BEAT)
                    .enable()
                    .set_bit()
            });
        }
        Ok(DmacTransfer {
            channel: self,
            buffer: source,
        })
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum TRIGSRC {
    DISABLE = 0x00,
    RTC_TIMESTAMP = 0x01,
    DSU_DCC0 = 0x02,
    DSU_DCC1 = 0x03,
    SERCOM0_RX = 0x04,
    SERCOM0_TX = 0x05,
    SERCOM1_RX = 0x06,
    SERCOM1_TX = 0x07,
    SERCOM2_RX = 0x08,
    SERCOM2_TX = 0x09,
    SERCOM3_RX = 0x0A,
    SERCOM3_TX = 0x0B,
    SERCOM4_RX = 0x0C,
    SERCOM4_TX = 0x0D,
    SERCOM5_RX = 0x0E,
    SERCOM5_TX = 0x0F,
    SERCOM6_RX = 0x10,
    SERCOM6_TX = 0x11,
    SERCOM7_RX = 0x12,
    SERCOM7_TX = 0x13,
    CAN0_DEBUG = 0x14,
    CAN1_DEBUG = 0x15,
    TCC0_OVF = 0x16,
    TCC0_MC_0 = 0x17,
    TCC0_MC_1 = 0x18,
    TCC0_MC_2 = 0x19,
    TCC0_MC_3 = 0x1a,
    TCC0_MC_4 = 0x1b,
    TCC0_MC_5 = 0x1c,
    TCC1_OVF = 0x1d,
    TCC1_MC_0 = 0x1e,
    TCC1_MC_1 = 0x1f,
    TCC1_MC_2 = 0x20,
    TCC1_MC_3 = 0x21,
    TCC2_OVF = 0x22,
    TCC2_MC_0 = 0x23,
    TCC2_MC_1 = 0x24,
    TCC2_MC_2 = 0x25,
    TCC3_OVF = 0x26,
    TCC3_MC_0 = 0x27,
    TCC3_MC_1 = 0x28,
    TCC4_OVF = 0x29,
    TCC4_MC_0 = 0x2a,
    TCC4_MC_1 = 0x2b,
    TC0_OVF = 0x2c,
    TC0_MC_0 = 0x2d,
    TC0_MC_1 = 0x2e,
    TC1_OVF = 0x2f,
    TC1_MC_0 = 0x30,
    TC1_MC_1 = 0x31,
    TC2_OVF = 0x32,
    TC2_MC_0 = 0x33,
    TC2_MC_1 = 0x34,
    TC3_OVF = 0x35,
    TC3_MC_0 = 0x36,
    TC3_MC_1 = 0x37,
    TC4_OVF = 0x38,
    TC4_MC_0 = 0x39,
    TC4_MC_1 = 0x3a,
    TC5_OVF = 0x3b,
    TC5_MC_0 = 0x3c,
    TC5_MC_1 = 0x3d,
    TC6_OVF = 0x3e,
    TC6_MC_0 = 0x3f,
    TC6_MC_1 = 0x40,
    TC7_OVF = 0x41,
    TC7_MC_0 = 0x42,
    TC7_MC_1 = 0x43,
    ADC0_RESRDY = 0x44,
    ADC0_SEQ = 0x45,
    ADC1_RESRDY = 0x46,
    ADC1_SEQ = 0x47,
    DAC_EMPTY_0 = 0x48,
    DAC_EMPTY_1 = 0x49,
    DAC_RESRDY_0 = 0x4A,
    DAC_RESRDY_1 = 0x4b,
    I2S_RX_0 = 0x4C,
    I2S_RX_1 = 0x4d,
    I2S_TX_0 = 0x4E,
    I2S_TX_1 = 0x4f,
    PCC_RX = 0x50,
    AES_WR = 0x51,
    AES_RD = 0x52,
    QSPI_RX = 0x53,
    QSPI_TX = 0x54,
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
        unsafe {
            Self {
                btctrl: core::mem::zeroed(),
                btcnt: 0,
                srcaddr: 0,
                dstaddr: 0,
                descaddr: 0,
            }
        }
    }

    unsafe fn set_srcaddr<Word: WordType>(
        self: &mut Self,
        buffer: *const Word,
        transfer_count: u16,
    ) -> &mut Self {
        self.srcaddr = (buffer as u32) + ((transfer_count as usize) * Word::word_size()) as u32;
        self
    }
    unsafe fn set_dstaddr<Word: WordType>(
        self: &mut Self,
        buffer: *const Word,
        transfer_count: u16,
    ) -> &mut Self {
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

pub trait WordType {
    fn beatsize() -> BEATSIZE;
    fn word_size() -> usize;
}
impl WordType for u8 {
    #[inline(always)]
    fn word_size() -> usize {
        1
    }
    #[inline(always)]
    fn beatsize() -> BEATSIZE {
        BEATSIZE::BYTE
    }
}
impl WordType for u16 {
    #[inline(always)]
    fn word_size() -> usize {
        2
    }
    #[inline(always)]
    fn beatsize() -> BEATSIZE {
        BEATSIZE::HWORD
    }
}
impl WordType for u32 {
    #[inline(always)]
    fn word_size() -> usize {
        4
    }
    #[inline(always)]
    fn beatsize() -> BEATSIZE {
        BEATSIZE::WORD
    }
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
        fn $getter_name(self: &Self) -> $field_type {
            ((self.value >> $bit) & ((1u16 << $bit) - 1u16))
                .try_into()
                .unwrap()
        }
        fn $setter_name(self: &mut Self, value: $field_type) -> &mut Self {
            self.value = (self.value & !(((1u16 << $bit) - 1u16) << $bit))
                | (((value as u16) & ((1u16 << $bit) - 1u16)) << $bit);
            self
        }
    };
    ($is_name:ident, $set_name:ident, $clear_name:ident: ($bit:expr)) => {
        fn $is_name(self: &Self) -> bool {
            (self.value & (1u16 << $bit)) != 0
        }
        fn $set_name(self: &mut Self) -> &mut Self {
            self.value |= (1u16 << $bit);
            self
        }
        fn $clear_name(self: &mut Self) -> &mut Self {
            self.value &= !(1u16 << $bit);
            self
        }
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

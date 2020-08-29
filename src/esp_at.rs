#![no_std]

use embedded_hal::serial;
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::{OutputPin, InputPin};

use heapless::ArrayLength;
use heapless::spsc::{Queue, Producer, Consumer};

#[derive(Debug, Copy, Clone)]
pub enum TransportError {
    Busy,
}

type ReadQueue = Queue<u8, heapless::consts::U256, u8>;
type WriteQueue = Queue<u8, heapless::consts::U256, u8>;

pub struct WioTerminalSPITransport<Spi, CsPin, IrqPin, SyncPin> 
    where   Spi: spi::Transfer<u8> + spi::Write<u8>,
            CsPin: OutputPin,
            IrqPin: InputPin,
            SyncPin: InputPin,
{
    spi: Spi,
    cs_pin: CsPin,
    irq_pin: IrqPin,
    sync_pin: SyncPin,
    rq: ReadQueue,
    wq: WriteQueue,
    state: State,
    buffer: [u8; 1024],
}

#[derive(Debug, Copy, Clone)]
enum State {
    Idle,
    IssueReadData,
    ReadDataResponseHeader {index: u8},
    ReadDataResponseLength,
    ReadDataResponseBody {remaining: u16, in_buffer_len: u16, in_buffer_offset: u16},
    IssueWriteData,
}

const SPT_TAG_PRE: u8 = 0x55u8; /* Master initiate a TRANSFER */
const SPT_TAG_WR: u8 = 0x80u8;  /* Master WRITE  to Slave */
const SPT_TAG_RD: u8 = 0x00u8;  /* Master READ from Slave */
const SPT_TAG_ACK: u8 = 0xBEu8; /* Slave  Acknowledgement */
const SPT_TAG_DMY: u8 = 0xFFu8; /* dummy */
const SPT_ERR_OK: u8 = 0x00u8;

struct ScopedGuard<Func: FnMut()> {
    func: Func,
    suppressed: bool,
}
impl <Func: FnMut()> ScopedGuard<Func> {
    pub fn new(func: Func) -> Self {
        Self {
            func: func,
            suppressed: false,
        }
    }
    pub fn suppress(self: &mut Self) {
        self.suppressed = true;
    }
}
impl <Func: FnMut()> Drop for ScopedGuard<Func> {
    fn drop(&mut self) {
        if !self.suppressed {
            (self.func)();
        }
    }
}

impl <Spi, CsPin, IrqPin, SyncPin>  WioTerminalSPITransport<Spi, CsPin, IrqPin, SyncPin> 
    where   Spi: spi::Transfer<u8> + spi::Write<u8>,
            CsPin: OutputPin,
            IrqPin: InputPin,
            SyncPin: InputPin,
{
    pub fn new(spi: Spi, cs_pin: CsPin, irq_pin: IrqPin, sync_pin: SyncPin) -> Self {
        Self {
            spi: spi,
            cs_pin: cs_pin,
            irq_pin: irq_pin,
            sync_pin: sync_pin,
            rq: Queue::u8(),
            wq: Queue::u8(),
            state: State::Idle,
            buffer: [0u8; 1024],
        }
    }
    
    #[inline(always)]
    fn assert_cs(self: &mut Self) {
        self.cs_pin.set_low();
    }
    #[inline(always)]
    fn deassert_cs(self: &mut Self) {
        self.cs_pin.set_high();
    }

    pub fn process(self: &mut Self) {
        loop {
            match self.state {
                State::Idle => {
                    self.deassert_cs();
                    let wq = self.wq.split().1;
                    if wq.ready() {
                        self.state = State::IssueWriteData;
                    } else if let Ok(pin) = self.irq_pin.is_high() {
                        if pin {
                            self.state = State::IssueReadData;
                        }
                        else {
                            break;
                        }
                    } else {
                        break;
                    }
                },
                State::IssueReadData => {
                    let command = [SPT_TAG_PRE, SPT_TAG_RD];
                    self.assert_cs();
                    let _ = ScopedGuard::new(|| { self.deassert_cs() });
                    if self.spi.write(&command).is_err() {
                        // Failed to transmit.
                        self.state = State::Idle;
                        return;
                    }
                    self.state = State::ReadDataResponseHeader {index: 0};
                },
                State::ReadDataResponseHeader {mut index} => {
                    let mut buffer = [0u8; 2];
                    self.assert_cs();
                    let _ = ScopedGuard::new(|| { self.deassert_cs() });
                    let result = self.spi.transfer(&mut buffer[index as usize..]);
                    if result.is_err() {
                        self.state = State::Idle;
                        return;
                    }
                    if index == 0 {
                        // Search TAG_ACK
                        if buffer[0] == SPT_TAG_ACK {
                            index = 1;
                        }
                        else if buffer[1] == SPT_TAG_ACK {
                            self.state = State::ReadDataResponseHeader {index: 1};
                        }
                        else {
                            break;
                        }
                    }
                    if index == 1 {
                        if buffer[1] == SPT_ERR_OK {
                            self.state = State::ReadDataResponseLength;
                        }
                        else {
                            // Error response
                            self.state = State::Idle;
                        }
                    }
                },
                State::ReadDataResponseLength => {
                    let mut buffer = [0u8; 2];
                    self.assert_cs();
                    let _ = ScopedGuard::new(|| { self.deassert_cs() });
                    let result = self.spi.transfer(&mut buffer);
                    if result.is_err() {
                        self.state = State::Idle;
                        return;
                    }
                    let length = (buffer[0] as u16) << 8 | (buffer[1] as u16);
                    self.state = State::ReadDataResponseBody {remaining: length, in_buffer_len: 0, in_buffer_offset: 0};
                },
                State::ReadDataResponseBody {mut remaining, mut in_buffer_len, mut in_buffer_offset } => {
                    if in_buffer_len == 0 {
                        // If there are no data in the buffer, read from the target.
                        self.assert_cs();
                        let _ = ScopedGuard::new(|| { self.deassert_cs() });
                        let bytes_to_read = core::cmp::min(self.buffer.len(), remaining as usize) as u16;
                        let result = self.spi.transfer(&mut self.buffer[..bytes_to_read as usize]);
                        if result.is_err() {
                            self.state = State::Idle;
                            return;
                        }
                        in_buffer_len = bytes_to_read;
                        in_buffer_offset = 0;
                        remaining -= bytes_to_read;
                    }
                    
                    // Enqueue data in the buffer
                    let mut rq = self.rq.split().0;
                    while in_buffer_offset < in_buffer_len {
                        if let Err(e) = rq.enqueue(self.buffer[in_buffer_offset as usize]) {
                            break
                        }
                        in_buffer_offset += 1;
                    }

                    // If there are no remaining data on the target and the local buffer, return to idle state.
                    if remaining == 0 && in_buffer_offset == in_buffer_len {
                        self.state = State::Idle;
                    }
                    else {
                        self.state = State::ReadDataResponseBody { remaining, in_buffer_len, in_buffer_offset };
                    }
                },
                State::IssueWriteData => {
                    let mut bytes_to_write = 0usize;
                    let mut wq = self.wq.split().1;
                    while wq.ready() && bytes_to_write < self.buffer.len() {
                        self.buffer[bytes_to_write] = wq.dequeue().unwrap();
                        bytes_to_write += 1;
                    }
                    
                    self.buffer[0] = SPT_TAG_PRE;
                    self.buffer[1] = SPT_TAG_WR;
                    self.buffer[2] = (bytes_to_write & 0xff) as u8;
                    self.buffer[3] = (bytes_to_write >> 8) as u8;
                    
                    self.assert_cs();
                    let _ = ScopedGuard::new(|| self.deassert_cs() );
                    let result = self.spi.write(&self.buffer);
                    if result.is_err() {
                        self.state = State::Idle;
                        return;
                    }
                    self.state = State::Idle;
                },
            }
        }
    }

    pub fn split<'queue>(&'queue mut self) -> (WioTerminalSPITransportTx<'queue, heapless::consts::U256>, WioTerminalSPITransportRx<'queue, heapless::consts::U256>) {
        (
            WioTerminalSPITransportTx { wq: self.wq.split().0 },
            WioTerminalSPITransportRx { rq: self.rq.split().1 },
        )
    }
}

pub struct WioTerminalSPITransportRx<'queue, N>  
    where N: ArrayLength<u8>
{
    rq: Consumer<'queue, u8, N, u8>,
}

impl<'queue, N> serial::Read<u8> for WioTerminalSPITransportRx<'queue, N>
    where N: ArrayLength<u8>
{
    type Error = TransportError;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if let Some(v) = self.rq.dequeue() {
            Ok(v)
        }
        else {
            Err(nb::Error::WouldBlock)
        }
    }
}

pub struct WioTerminalSPITransportTx<'queue, N> 
    where N: ArrayLength<u8>
{
    wq: Producer<'queue, u8, N, u8>
}

impl<'queue, N> serial::Write<u8> for WioTerminalSPITransportTx<'queue, N>
    where N: ArrayLength<u8>
{
    type Error = TransportError;
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.wq.enqueue(word).map_err(|_| nb::Error::WouldBlock)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}
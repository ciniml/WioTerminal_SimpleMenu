#![no_std]

use embedded_hal::spi::{FullDuplex};
use core::pin::Pin;
use embedded_dma::*;
use atsamd_hal::spi_common::CommonSpi;
use atsamd_hal::clock::GenericClockController;
use atsamd51p19a::{DMAC, MCLK, dmac::channel::chctrla::*};
use core::{convert::TryFrom, convert::TryInto, cell::RefCell};

use crate::atsamd_dmac::*;
use crate::async_operation::*;

#[derive(Debug, Copy, Clone)]
pub enum ESPTransportError {
    Busy,
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

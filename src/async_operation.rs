#![no_std]

use embedded_hal::spi::{FullDuplex};
use core::pin::Pin;
use embedded_dma::*;
use atsamd_hal::spi_common::CommonSpi;
use atsamd_hal::clock::GenericClockController;
use atsamd51p19a::{DMAC, MCLK, dmac::channel::chctrla::*};
use core::{convert::TryFrom, convert::TryInto, cell::RefCell};

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

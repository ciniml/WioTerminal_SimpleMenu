#![no_std]









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

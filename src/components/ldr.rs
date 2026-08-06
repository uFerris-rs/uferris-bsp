/// A trait for reading a raw ADC value.
pub trait OneShot {
    fn read_raw(&mut self) -> u16;
}

/// The `async` counterpart of [`OneShot`].
///
/// Board adapters implement this for their ADC wrapper when the controller HAL
/// drives the converter through a future — which is the normal case for the
/// embassy HALs, where the blocking board has to poll the peripheral registers
/// by hand instead.
#[cfg(feature = "async")]
#[allow(
    async_fn_in_trait,
    reason = "the board adapters are the only implementors and the only caller \
    is the board itself, so no auto trait bound on the returned future is needed."
)]
pub trait OneShotAsync {
    async fn read_raw(&mut self) -> u16;
}

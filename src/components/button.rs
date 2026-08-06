use core::marker::PhantomData;

#[cfg(feature = "async")]
use crate::Async;
use crate::{Blocking, Mode};

/// A push button wired straight to a controller pin.
///
/// `M` selects the board mode this button belongs to: see [`Mode`]. The pin
/// bounds live on the `impl` blocks rather than on the struct, so that a board
/// type can be named without them.
///
/// Reading the pin in [`Blocking`] mode goes through [`Button::pin`] directly,
/// the way the board has always done it. `Async` mode adds
/// `Button::wait_for_press`, which suspends rather than spins.
pub struct Button<PIN, M = Blocking> {
    pub pin: PIN,
    _mode: PhantomData<M>,
}

impl<PIN, M: Mode> Button<PIN, M> {
    pub fn new(pin: PIN) -> Self {
        Self {
            pin,
            _mode: PhantomData,
        }
    }
}

#[cfg(feature = "async")]
impl<PIN: embedded_hal_async::digital::Wait> Button<PIN, Async> {
    /// Wait until the button is pressed.
    ///
    /// The uFerris push buttons are active low, so this waits for the pin to go
    /// low. It waits on the level rather than on an edge, so a button that is
    /// already held down returns immediately — matching what a poll loop over
    /// the blocking read would do.
    pub async fn wait_for_press(&mut self) {
        let _ = self.pin.wait_for_low().await;
    }
}

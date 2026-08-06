/// An LED wired straight to a controller pin.
///
/// The `OutputPin` bound lives on the board `impl` blocks that drive the pin
/// rather than on this struct, so that a board type can be named without it.
pub struct Led<PIN> {
    pub pin: PIN,
}

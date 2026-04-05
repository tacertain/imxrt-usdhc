//! `SdHost` trait: the hardware abstraction boundary between the SD protocol
//! layer and raw USDHC register access.
//!
//! The protocol code (`card_init`, `read_csd`, `set_bus_width_4bit`,
//! `read_single_block`, `write_single_block`) is written against this trait
//! so that tests can substitute a mock implementation without touching
//! real hardware.

use crate::cmd::UsdCmdFlags;
use crate::error::SdError;

/// Response from a command.
///
/// For R1/R3/R6/R7 (48-bit) responses, only `rsp0` is valid.
///
/// For R2 (136-bit) responses, all four fields are valid and contain
/// the CID or CSD register data **after normalization**: the hardware
/// stores R2 responses with the internal CRC and end bit stripped and
/// the remaining bits shifted. The `SdHost` implementation must apply
/// the 8-bit left-shift correction so that the protocol layer receives
/// data aligned to the SD specification's CID/CSD bit numbering.
///
/// Normalization (performed by the `SdHost` impl, not the caller):
/// ```text
/// rsp[3] = (rsp[3] << 8) | (rsp[2] >> 24);
/// rsp[2] = (rsp[2] << 8) | (rsp[1] >> 24);
/// rsp[1] = (rsp[1] << 8) | (rsp[0] >> 24);
/// rsp[0] = rsp[0] << 8;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CmdResponse {
    pub rsp0: u32,
    pub rsp1: u32,
    pub rsp2: u32,
    pub rsp3: u32,
}

/// SD bus width.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BusWidth {
    /// 1-bit mode (DAT0 only). Used during identification.
    One,
    /// 4-bit mode (DAT0-DAT3). Used after initialization.
    Four,
}

/// Minimal hardware interface for the SD protocol layer.
///
/// Each method maps to a sequence of register operations. The trait
/// exists so that tests can substitute a mock implementation.
///
/// Methods take `&self` because the real implementation accesses hardware
/// through raw pointers (inherently interior-mutable). Test mocks use
/// `RefCell` for their mutable state.
pub(crate) trait SdHost {
    /// Send a command and return the response registers.
    fn send_cmd(
        &self,
        cmd_index: u32,
        arg: u32,
        flags: UsdCmdFlags,
    ) -> Result<CmdResponse, SdError>;

    /// Issue a read data command, transfer 512 bytes from the data port
    /// into `buf`, and wait for transfer complete.
    fn read_block(
        &self,
        cmd_index: u32,
        arg: u32,
        buf: &mut [u8; 512],
    ) -> Result<(), SdError>;

    /// Issue a write data command, transfer 512 bytes from `buf` to the
    /// data port, and wait for transfer complete.
    fn write_block(
        &self,
        cmd_index: u32,
        arg: u32,
        buf: &[u8; 512],
    ) -> Result<(), SdError>;

    /// Switch the SD clock to the given frequency (or lower).
    ///
    /// The implementation handles any clock-gating required during the
    /// divisor change (e.g. FRC_SDCLK_ON toggling).
    fn set_clock(&self, target_hz: u32);

    /// Set the bus width at the hardware level (write PROT_CTRL.DTW).
    fn set_bus_width(&self, width: BusWidth);

    /// Perform a software reset of the USDHC peripheral and apply
    /// initial register configuration (timeouts, watermarks, status
    /// enables, endian mode, etc.).
    ///
    /// This does NOT perform a hardware power cycle of the card.
    /// Card power sequencing and the hardware reset line (IPP_RST_N)
    /// are the BSP's responsibility.
    fn reset_and_configure(&self);

    /// Send the 80 initialization clock cycles.
    fn send_initial_clocks(&self);
}
